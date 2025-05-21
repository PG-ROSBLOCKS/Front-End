import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { LandingStateService } from '../services/landing-state.service';
import { CodeService } from '../services/code.service';
import { navigationPerf } from '../utilities/perf-utils';

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
    navigationPerf.clear();                      // limpio marcas previas
    navigationPerf.mark('get_started_click');
    this.landingState.markVisited();
    this.codeService.resetSession();
    this.router.navigate(['/loading']);
  }
}
