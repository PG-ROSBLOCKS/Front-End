// cypress/e2e/asr1-prod.cy.ts

describe('ASR-1 – client/server flow OK (producción)', () => {
  /* ─────────── CONSTANTES CONFIGURABLES ─────────── */
  const FRONT_URL       = 'http://d1imdbog9ssvrx.cloudfront.net/';
  const GET_STARTED_BTN = 'button.action-btn.primary';
  const LOAD_WS_BTN     = 'button.load';
  const ROSBLOCKS_FILE  = 'cypress/fixtures/asr_2.rosblocks';
  const SERVICE_NAME    = 'AddTwoInts.srv';
  const SERVICE_BODY    = {
    file_name: SERVICE_NAME,
    code: `# Archivo AddTwoInts.srv generado por ROSBlocks
int64 a
int64 b
---
int64 sum`,
    type: 'srv'
  };
  const SLA             = 50_000;        // ms
  const POLLER_URL      = 'http://44.222.251.149/api/get-ip/*';
  const POLL_TIMEOUT    = 60_000;        // ms máximos para polling

  /**
   * Lanza el polling al endpoint POLLER_URL hasta recibir {status:"ready", ip}.
   * Guarda la URL resultante en CYPRESS_API_URL.
   */
  function captureApiUrl(): Cypress.Chainable<string> {
    const cached = Cypress.env('API_URL');
    if (cached) {
      return cy.wrap(cached);
    }

    cy.intercept('GET', POLLER_URL).as('poll');

    function waitReady(): Cypress.Chainable<string> {
      return cy
        .wait('@poll', { timeout: POLL_TIMEOUT })
        .then(inter => {
          const { status, ip } =
            inter.response!.body as { status: string; ip?: string };
          if (status === 'ready' && ip) {
            const api = `http://${ip}:8000`;
            Cypress.env('API_URL', api);
            return cy.wrap(api);
          }
          // si aún no está listo, reintenta
          return waitReady();
        });
    }

    return waitReady();
  }

  /* ─────────── 1. PREPARACIÓN GLOBAL ─────────── */
  before(() => {
    // 1-A: visita landing
    cy.visit(FRONT_URL);

    // 1-B: arranca el interceptor de polling **antes** de hacer clic
    cy.intercept('GET', POLLER_URL).as('poll');

    // 1-C: haz clic en Get Started y espera redirección a /workspace
    cy.get(GET_STARTED_BTN, { timeout: 60_000 })
      .should('be.visible')
      .click({ force: true });
    cy.url({ timeout: 70_000 }).should('include', '/workspace');

    // 1-D: ahora sí espera al poller y sube el .srv de prueba
    captureApiUrl().then(api => {
      cy.request({
        method: 'POST',
        url:    `${api}/upload/`,
        headers:{ 'Content-Type': 'application/json' },
        body:   SERVICE_BODY,
        failOnStatusCode: false
      }).its('status').should('eq', 200);
    });
  });

  /* ─────────── 2. PREPARACIÓN POR CASO ─────────── */
  beforeEach(() => {
    cy.visit(FRONT_URL);
    cy.get(GET_STARTED_BTN, { timeout: 30_000 }).click();
    cy.url({ timeout: 30_000 }).should('include', '/workspace');

    // pulsar Load workspace
    cy.get(LOAD_WS_BTN, { timeout: 30_000 })
      .should('be.visible')
      .click();

    // **Solo** el input de .rosblocks (evita el de export)
    cy.get('input[type="file"][accept=".rosblocks"]', { timeout: 30_000 })
      .first()
      .selectFile(ROSBLOCKS_FILE, { force: true });

    // 3) CLICK en el diálogo de confirmación "Data loaded successfully"
    cy.contains('.confirmDialog p', 'Data loaded successfully').should('be.visible');
    cy.get('button.btnConfirm').click();
    // espera a que desaparezca la overlay
    cy.get('.overlay').should('not.exist');

    // al menos dos pestañas cargadas
    cy.get('li.nodeTab', { timeout: 60_000 })
      .should('have.length.at.least', 2);
  });

  /* ─────────── 3. CASO DE PRUEBA ─────────── */
  function ensureAllRunning() {
    cy.get('button.playButton', { timeout: 10_000 }).then($btns => {
      const pendientes = [...$btns].filter(
        el => el.innerText.trim() === '▶'
      );
      if (pendientes.length) {
        cy.wrap(pendientes).each(el =>
          cy.wrap(el).click({ force: true })
        );
        cy.wait(150);
        ensureAllRunning();
      }
    });
  }

  it(`muestra “Respuesta del servidor:” en menos de ${SLA} ms`, () => {
    // 3-A: selecciona la pestaña “cliente”
    cy.get('input.node', { timeout: 30_000 })
      .filter((_, el) =>
        el.value.trim().toLowerCase() === 'cliente'
      )
      .first()
      .parents('li.nodeTab')
      .click({ force: true });

    // 3-B: marca t0 y pulsa Play All
    cy.then(() => Date.now()).as('t0');
    cy.get('button.playAll')
      .should('contain.text', '▶')
      .click({ force: true });

    // 3-C: fuerza todos los ▶ a ◼
    ensureAllRunning();

    // 3-D: espera la línea “Respuesta del servidor:”
    cy.contains(
      'span.console_lines',
      'Respuesta del servidor:',
      { timeout: SLA }
    ).then(function () {
      const elapsed = Date.now() - (this as any).t0;
      cy.log(`Latencia medida: ${elapsed} ms`);
      expect(elapsed).to.be.lessThan(SLA);
    });
  });

  /* ─────────── 4. LIMPIEZA GLOBAL ─────────── */
  after(() => {
    // 4-A: borra el .srv usando la misma API dinámica
    captureApiUrl().then(api => {
      cy.request({
        method: 'DELETE',
        url:    `${api}/delete/interfaces/srv/${SERVICE_NAME}`,
        failOnStatusCode: false
      });
    });
    // 4-B: detiene todos los nodos
    cy.get('button.stopAll', { timeout: 0 }).then($btn => {
      if ($btn.length) cy.wrap($btn).click({ force: true });
    });
  });
});
