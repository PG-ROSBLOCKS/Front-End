import {
  RouteReuseStrategy,
  DetachedRouteHandle,
  ActivatedRouteSnapshot
} from '@angular/router';
import { Injectable } from '@angular/core';

@Injectable()
export class SimpleReuseStrategy implements RouteReuseStrategy {

  private handlers: Record<string, DetachedRouteHandle> = {};

  // ─── decide si se debe desprender el componente ─────────────────────────────
  shouldDetach(route: ActivatedRouteSnapshot): boolean {
    return route.data?.['reuse'] === true;
  }

  // ─── guarda la instancia desprendida ────────────────────────────────────────
  store(route: ActivatedRouteSnapshot, handle: DetachedRouteHandle): void {
    this.handlers[this.getKey(route)] = handle;
  }

  // ─── decide si existe una instancia para volver a usar ──────────────────────
  shouldAttach(route: ActivatedRouteSnapshot): boolean {
    return !!this.handlers[this.getKey(route)];
  }

  // ─── devuelve la instancia guardada ─────────────────────────────────────────
  retrieve(route: ActivatedRouteSnapshot): DetachedRouteHandle | null {
    return this.handlers[this.getKey(route)] || null;
  }

  // ─── conserva la misma instancia para la misma configuración de ruta ───────
  shouldReuseRoute(future: ActivatedRouteSnapshot,
                   curr:   ActivatedRouteSnapshot): boolean {
    return future.routeConfig === curr.routeConfig;
  }

  // helper
  private getKey(route: ActivatedRouteSnapshot): string {
    // usa la ruta completa ('' para el wildcard **)
    return route.routeConfig?.path ?? '';
  }
}