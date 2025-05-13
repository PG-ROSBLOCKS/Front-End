import { Injectable, OnDestroy } from '@angular/core';
import { Router, NavigationEnd, Event } from '@angular/router';
import { HttpClient } from '@angular/common/http';
import { AlertService } from '../shared/components/alert/alert.service';
import { Subscription, interval, switchMap, catchError, of } from 'rxjs';
import { filter, take } from 'rxjs/operators';
import { CodeService } from './code.service';
import { globalMonitorPerf, PerfTest } from '../utilities/perf-utils';
import { safeUUID } from '../utilities/sanitizer-tools';

@Injectable({ providedIn: 'root' })
export class BackendMonitorService implements OnDestroy {
  private readonly HEARTBEAT_INTERVAL = 10_000;
  private heartbeatCount = 0;
  private sub?: Subscription;
  private routerSub: Subscription;
  private isRunning = false;

  constructor(
    private http: HttpClient,
    private alert: AlertService,
    private router: Router,
    private codeService: CodeService
  ) {
    // 1) cuando el CodeService avisa que ya tiene URL:
    this.codeService.ready$
      .pipe(
        filter(ready => ready),
        take(1)
      )
      .subscribe(() => {
        // sólo arrancamos el heartbeat si no estamos en “/”
        if (this.router.url !== '/') {
          console.log('[BackendMonitor] backend URL ready: ', this.codeService.apiUrl);
          this.startHeartbeat();
        }
      });

    // 2) nos suscribimos a cambios de ruta para parar/arrancar heartbeat
    this.routerSub = this.router.events
      .pipe(
        filter((e: Event): e is NavigationEnd => e instanceof NavigationEnd)
      )
      .subscribe(e => {
        if (e.urlAfterRedirects === '/') {
          this.stopHeartbeat();
        } else {
          this.startHeartbeat();
        }
      });
  }

  /** Arranca el polling periódico al /health del backend dinámico */
  public startHeartbeat(): void {
    globalMonitorPerf.clear();

    if (this.isRunning) { return; }
    this.isRunning = true;
    this.heartbeatCount = 0;
    console.log('[BackendMonitor] heartbeat STARTED');

    this.sub = interval(this.HEARTBEAT_INTERVAL)
      .pipe(
        switchMap(() =>
          this.http
            .get<{ status: string }>(
              `${this.codeService.apiUrl}/health/`
            )
            .pipe(
              catchError(err => {
                console.error('[BackendMonitor] health-check ERR:', err);
                return of({ status: 'down' });
              })
            )
        )
      )
      .subscribe(res => {
        this.heartbeatCount++;
        console.log(`[BackendMonitor] #${this.heartbeatCount} → status=${res.status}`);
        if (res.status !== 'ok') {
          globalMonitorPerf.mark('backend_down');
          globalMonitorPerf.measure('ws_to_backendDown', 'ws_error', 'backend_down');
          console.warn(`[BackendMonitor] backend DOWN @ #${this.heartbeatCount}`);
          this.onBackendDown();
        }
      });
  }

  /** Para el polling */
  private stopHeartbeat(): void {
    if (this.sub) {
      this.sub.unsubscribe();
      this.sub = undefined;
    }
    this.isRunning = false;
    console.log('[BackendMonitor] heartbeat STOPPED');
  }

  /** Cuando detectamos backend caído */
  private onBackendDown(): void {
    this.stopHeartbeat();
    this.alert.showAlert('El servidor backend no responde. Redirigiendo…');
    this.router.navigate(['/']);
    const m = globalMonitorPerf.getMeasures()
      .find(x => x.name === 'global:ws_to_backendDown');
    if (m) {
      console.table([{
        test: 'ws_to_backendDown',
        duration: `${m.duration.toFixed(2)} ms`
      }]);
    }
    localStorage.removeItem('uuid');
    localStorage.setItem('uuid', safeUUID());
    window.location.reload();

  }

  ngOnDestroy(): void {
    this.stopHeartbeat();
    this.routerSub.unsubscribe();
  }
}
