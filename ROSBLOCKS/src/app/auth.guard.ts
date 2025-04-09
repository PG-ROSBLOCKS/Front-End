import { Injectable } from '@angular/core';
import { environment } from '../environments/environment.development';
import {
  CanActivate,
  ActivatedRouteSnapshot,
  RouterStateSnapshot,
  UrlTree,
  Router,
} from '@angular/router';

@Injectable({
  providedIn: 'root',
})
export class AuthGuard implements CanActivate {
  constructor(private router: Router) {}

  canActivate(route: ActivatedRouteSnapshot, state: RouterStateSnapshot): boolean | UrlTree {
    const token = localStorage.getItem('access_token');
    if (token) {
      // Tenemos token => dejar pasar
      return true;
    } else {
      // No hay token => redirigir al login de JupyterHub
      // Ajusta esta URL con tu client_id, redirect_uri y state
      const stateParam = crypto.randomUUID();
      const authUrl = `http://${environment.backendFastAPIAddress}/hub/api/oauth2/authorize?client_id=${environment.backendClientId}&redirect_uri=${environment.backendRedirectUri}&response_type=code&state=${stateParam}`;

      window.location.href = authUrl;
      return false;
    }
  }
}
