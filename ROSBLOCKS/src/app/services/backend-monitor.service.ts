import { HttpClient, HttpErrorResponse, HttpEvent, HttpHandler, HttpInterceptor, HttpRequest } from '@angular/common/http';
import { Injectable, OnDestroy } from '@angular/core';
import { Router } from '@angular/router';
import { Subscription, interval, switchMap, catchError, of, filter, take, Observable, throwError } from 'rxjs';
import { AlertService } from '../shared/components/alert/alert.service';

@Injectable({
  providedIn: 'root'
})
export class BackendMonitorService implements OnDestroy {
    private readonly HEARTBEAT_INTERVAL = 5_000;
    private sub?: Subscription;
    private hasAlerted = false;       // <- flag para no repetir alertas
  
    constructor(
      private http: HttpClient,
      private alert: AlertService,
      private router: Router
    ) {
      this.startHeartbeat();
    }
  
    private notifyOnce(message: string) {
      if (!this.hasAlerted) {
        this.hasAlerted = true;
        this.alert.showAlert(message);
      }
    }
  
    private startHeartbeat(): void {
      this.sub = interval(this.HEARTBEAT_INTERVAL).pipe(
        switchMap(() =>
          this.http.get<{status:string}>('/health').pipe(
            catchError(() => of({ status: 'down' }))
          )
        ),
        take(1)
      ).subscribe(res => {
        if (res.status === 'ok') {
          // si está ok, reinicio flag y vuelvo a arrancar el heartbeat
          this.hasAlerted = false;
          this.startHeartbeat();
        } else {
          // si está down, sólo alerto una vez y redirijo
          this.notifyOnce('El servidor backend no responde. Redirigiendo…');
          this.router.navigate(['/']);
        }
      });
    }
  
    ngOnDestroy(): void {
      this.sub?.unsubscribe();
    }
  }
  