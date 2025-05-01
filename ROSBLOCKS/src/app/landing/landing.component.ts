import { Component } from '@angular/core';
import { Router } from '@angular/router';

@Component({
  selector: 'app-landing',
  templateUrl: './landing.component.html',
  styleUrls: ['./landing.component.css']
})
export class LandingComponent {
  constructor(private router: Router) {}

  // Navigate to the tutorials section
  goToTutorials(): void {
    this.router.navigate(['/help']);
  }

  // Navigate to the workspace (root)
  start(): void {
    this.router.navigate(['/workspace']);
  }
}
