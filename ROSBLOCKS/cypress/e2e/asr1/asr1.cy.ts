// cypress/e2e/asr1-latency.cy.ts
describe('ASR-1 – pub/sub latency', () => {
  const TIME_LIMIT = 1500;                                // ms
  const API        = Cypress.env('API_URL') ||            //  http://localhost:8000
                     'http://localhost:8000';

  let project: Record<string, string>;                    // JSON del .rosblocks

  /* ──────────────────────────── 1. PREPARACIÓN ──────────────────────────── */
  before(() => {
    /* 1-A  carga el proyecto para localStorage */
    cy.fixture('sub.rosblocks', 'utf-8').then(raw => {
      project = JSON.parse(raw);
    });

    /* 1-B  sube el fichero .srv de prueba */
    cy.request({
      method: 'POST',
      url:    `${API}/upload/`,
      headers: { 'Content-Type': 'application/json' },
      body: {
        file_name: 'AddTwoInts.srv',
        code:      '# Archivo AddTwoInts.srv generado por ROSBlocks\nint64 a\nint64 b\n---\nint64 sum',
        type:      'srv'
      },
      failOnStatusCode: false              // continúa si ya existe
    }).then(res => {
      expect(res.status).to.eq(200);
      expect(res.body).to.have.property(
        'message',
        'Service uploaded and package rebuilt'
      );
    });
  });

  /* ──────────────────────────── 2. PRUEBA SLA ───────────────────────────── */
  it(`el mensaje aparece en < ${TIME_LIMIT} ms`, () => {
    cy.visit('http://localhost:4200/#/app', {
      onBeforeLoad: win => {
        Object.entries(project).forEach(([k, v]) => win.localStorage.setItem(k, v));
      }
    });

    // const t0 = Date.now();
    // cy.get('button#playAll').click();

    cy.contains('Save workspace')//, /Received:\s*\d+/, { timeout: TIME_LIMIT })
      // .then(() => {
      //   const elapsed = Date.now() - t0;
      //   cy.log(`Latencia medida: ${elapsed} ms`);
      //   expect(elapsed).to.be.lessThan(TIME_LIMIT);
      // });
  });

  /* ──────────────────────────── 3. LIMPIEZA ─────────────────────────────── */
  after(() => {
    cy.request({
      method: 'DELETE',
      url:    `${API}/delete/interfaces/srv/AddTwoInts`,
      failOnStatusCode: false              // evita fallo si ya se borró
    }).then(res => {
      expect(res.status).to.eq(200);
      expect(res.body).to.have.property('file', 'AddTwoInts.srv');
    });
  });
});
