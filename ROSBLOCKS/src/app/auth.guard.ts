import { Injectable } from '@angular/core';
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
      const clientId = 'service-fastapi';
      const redirectUri = encodeURIComponent('http://localhost:4200/oauth-callback');
      const stateParam = crypto.randomUUID();
      const authUrl = `http://34.58.80.154/hub/api/oauth2/authorize?client_id=${clientId}&redirect_uri=${redirectUri}&response_type=code&state=${stateParam}`;

      window.location.href = authUrl;
      return false;
    }
  }
}
