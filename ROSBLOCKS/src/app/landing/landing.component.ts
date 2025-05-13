import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { LandingStateService } from '../services/landing-state.service';
import { CodeService } from '../services/code.service';

@Component({
  selector: 'app-landing',
  templateUrl: './landing.component.html',
  styleUrls: ['./landing.component.css']
})
export class LandingComponent {
  constructor(private router: Router, private landingState: LandingStateService, private codeService: CodeService) {}

  goToTutorials(): void {
    this.router.navigate(['/help']);
  }

  start(): void {
    this.landingState.markVisited();
    localStorage.removeItem('uuid');
    //this.codeService.resetSession();
    this.router.navigate(['/loading']);
  }
}
