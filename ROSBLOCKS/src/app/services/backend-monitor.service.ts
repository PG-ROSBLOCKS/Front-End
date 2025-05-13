import { Injectable, OnDestroy } from '@angular/core';
import { Router, NavigationEnd, Event } from '@angular/router';
import { HttpClient } from '@angular/common/http';
import { AlertService } from '../shared/components/alert/alert.service';
import { Subscription, interval, switchMap, catchError, of } from 'rxjs';
import { filter } from 'rxjs/operators';

@Injectable({ providedIn: 'root' })
export class BackendMonitorService implements OnDestroy {
  private readonly HEARTBEAT_INTERVAL = 5_000;
  private heartbeatCount = 0;
  private sub?: Subscription;
  private routerSub: Subscription;
  private isRunning = false;

  constructor(
    private http: HttpClient,
    private alert: AlertService,
    private router: Router
  ) {
    // Escucho cambios de ruta y solo dejo pasar NavigationEnd
    this.routerSub = this.router.events
      .pipe(
        filter((ev): ev is NavigationEnd => ev instanceof NavigationEnd)
      )
      .subscribe(ev => {
        if (ev.urlAfterRedirects === '/') {
          this.stopHeartbeat();
        } else {
          this.startHeartbeat();
        }
      });

    // Arranco al init si no estamos en '/'
    if (this.router.url !== '/') {
      this.startHeartbeat();
    }
  }

  public startHeartbeat(): void {
    if (this.isRunning) { return; }
    this.isRunning = true;
    this.heartbeatCount = 0;
    console.log('[BackendMonitor] heartbeat started');

    this.sub = interval(this.HEARTBEAT_INTERVAL)
      .pipe(
        switchMap(() =>
          this.http.get<{ status: string }>('http://localhost:8000/health').pipe(
            catchError(err => {
              console.error('[BackendMonitor] Error en el health check:', err);
              return of({ status: 'down' });
            })
          )
        )
      )
      .subscribe(res => {
        this.heartbeatCount++;
        console.log(`[BackendMonitor] #${this.heartbeatCount} → status=${res.status}`);
        if (res.status !== 'ok') {
          console.warn(`[BackendMonitor] Backend down at heartbeat #${this.heartbeatCount}`);
          this.onBackendDown();
        }
      });
  }

  private stopHeartbeat(): void {
    if (this.sub) {
      this.sub.unsubscribe();
      this.sub = undefined;
    }
    this.isRunning = false;
    console.log('[BackendMonitor] heartbeat stopped');
  }

  private onBackendDown(): void {
    this.stopHeartbeat();
    this.alert.showAlert('El servidor backend no responde. Redirigiendo…');
    this.router.navigate(['/']);
  }

  ngOnDestroy(): void {
    this.stopHeartbeat();
    this.routerSub.unsubscribe();
  }
}
