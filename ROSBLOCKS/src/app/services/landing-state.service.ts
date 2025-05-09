import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class LandingStateService {

  private visited = false;

  markVisited() {
    this.visited = true;
    sessionStorage.setItem('visitedLanding', 'true');
  }

  hasVisited(): boolean {
    return this.visited
      || sessionStorage.getItem('visitedLanding') === 'true';
  }
}
