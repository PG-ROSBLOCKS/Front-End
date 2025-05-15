import { Component, HostListener } from '@angular/core';
import { Router } from '@angular/router';
import { LandingStateService } from '../services/landing-state.service';
import { CodeService } from '../services/code.service';

@Component({
  selector: 'app-landing',
  templateUrl: './landing.component.html',
  styleUrls: ['./landing.component.css']
})
export class LandingComponent {
  private lastActivity: number = Date.now();
  private intervalId: any;
  private userConnected: boolean = true

  isActive: boolean = true;
  
  constructor(private router: Router, private landingState: LandingStateService, private codeService: CodeService) {
    this.startInactivityCheck();
  }

  goToTutorials(): void {
    this.router.navigate(['/help']);
  }

  start(): void {
    this.landingState.markVisited();
    this.codeService.resetSession();
    this.router.navigate(['/loading']);
  }

  @HostListener('window:mousemove')
  @HostListener('window:keydown')
  @HostListener('window:click')
  @HostListener('window:scroll')
  resetInactivityTimer(): void {
    this.lastActivity = Date.now();
  }

  private startInactivityCheck(): void {
    this.intervalId = setInterval(() => {
      const now = Date.now();
      const diffInSeconds = (now - this.lastActivity) / 1000;

      if (diffInSeconds < 60 * 10) {
        this.userConnected = true
        console.log('ok');
      } else if (this.userConnected) {
        this.userConnected = false
        console.log('se ha desconectado');
      } else {
        console.log("Muerto");
        
      }
    }, 1000); // check every second
  }

  ngOnDestroy(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
    }
  }
}
