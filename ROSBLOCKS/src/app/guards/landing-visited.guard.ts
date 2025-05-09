// src/app/guards/landing-visited.guard.ts
import { Injectable } from '@angular/core';
import { CanActivate, Router, UrlTree } from '@angular/router';
import { LandingStateService } from '../services/landing-state.service';

@Injectable({ providedIn: 'root' })
export class LandingVisitedGuard implements CanActivate {
  constructor(
    private landingState: LandingStateService,
    private router: Router
  ) {}

  canActivate(): boolean | UrlTree {
    if (this.landingState.hasVisited()) {
      return true;
    }
    // si no ha visitado, redirige al landing
    return this.router.createUrlTree(['/']);
  }
}
