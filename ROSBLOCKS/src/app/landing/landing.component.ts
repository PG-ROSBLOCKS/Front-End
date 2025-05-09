import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { LandingStateService } from '../services/landing-state.service';

@Component({
  selector: 'app-landing',
  templateUrl: './landing.component.html',
  styleUrls: ['./landing.component.css']
})
export class LandingComponent {
  constructor(private router: Router, private landingState: LandingStateService) {}

  goToTutorials(): void {
    this.router.navigate(['/help']);
  }

  start(): void {
    this.router.navigate(['/loading']);
  }
}
