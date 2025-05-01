// cypress/e2e/asr1-basic.cy.ts
describe('ASR-1 – client/server flow OK', () => {
  const API = Cypress.env('API_URL') || 'http://localhost:8000';
  const SLA = 20000;

  /* 1.  PREPARACIÓN GLOBAL ─────────────────────────────────────────────── */
  before(() => {
    cy.request({
      method: 'POST',
      url: `${API}/upload/`,
      headers: { 'Content-Type': 'application/json' },
      body: {
        file_name: 'AddTwoInts.srv',
        code:
          `# Archivo AddTwoInts.srv generado por ROSBlocks
int64 a
int64 b
---
int64 sum`,
        type: 'srv'
      },
      failOnStatusCode: false
    });
  });

  /* 2.  PREPARACIÓN POR CASO ───────────────────────────────────────────── */
  beforeEach(() => {
    cy.fixture('asr_2.rosblocks', 'utf-8').then(raw => {
      const ws = JSON.parse(raw);

      cy.intercept('GET', `${API}/srvfiles*`).as('srvFiles');
      cy.intercept('GET', `${API}/msgfiles*`).as('msgFiles');

      cy.visit('http://localhost:4200/#/app', {
        onBeforeLoad(win) {
          Object.entries(ws).forEach(([k, v]) => win.localStorage.setItem(k, String(v)));
        }
      });

      cy.wait(['@srvFiles', '@msgFiles']);   // workspace totalmente cargado
    });
  });

  /* 3.  CASO DE PRUEBA ─────────────────────────────────────────────────── */
  /* 1. Asegúrate de que TODOS los playButton queden en “◼”.
        Si alguno sigue en “▶”, haz click individual y re-intenta. */
  function ensureAllRunning(): void {
    cy.get('button.playButton', { timeout: 10_000 }).then($btns => {
      const pendientes = [...$btns].filter(el => el.innerText.replace(/\s+/g, '') === '▶');

      if (pendientes.length) {
        cy.wrap(pendientes).each(el => cy.wrap(el).click({ force: true }));
        cy.wait(150);
        ensureAllRunning();                      // vuelve a comprobar
      }
    });
  }

  it(`el mensaje aparece en < ${SLA} ms`, () => {
    cy.get('input.node')
    .filter((_, el) => el.value.trim().toLowerCase() === 'cliente')
    .first()
    .parents('li.nodeTab')
    .click({ force: true });

  /* 1. marca t0 y pulsa Play All */
  cy.then(() => Date.now()).as('t0');
  cy.get('button.playAll')
    .should('contain.text', '▶')
    .click({ force: true });
    cy.get('button.playAll').should('contain.text', '▶').click({ force: true });

    /* ⇩ usa la función de arriba */
    ensureAllRunning();

    /* 2. Línea en consola */
    cy.contains('span.console_lines', 'Respuesta del servidor:', { timeout: SLA })
      .then(function () {
        const elapsed = Date.now() - (this.t0 as number);
        cy.log(`Latencia medida: ${elapsed} ms`);
        expect(elapsed).to.be.lessThan(SLA);
      });
  });

  /* 4.  LIMPIEZA GLOBAL ─────────────────────────────────────────────────── */
  after(() => {
    /* A) elimina el .srv de prueba (si existe) */
    cy.request({
      method: 'DELETE',
      url: `${API}/delete/interfaces/srv/AddTwoInts`,   // ← ruta correcta de tu endpoint
      failOnStatusCode: false                       // no falle si ya se borró
    });

    /* B) pulsa el botón “Stop All” para detener todos los nodos */
    cy.get('button.stopAll')                        // selector del botón
      .should('be.visible')
      .click({ force: true });
  })
});
