import { Component, OnDestroy, OnInit } from '@angular/core';
import { Router } from '@angular/router';
import { filter, Subscription, take, tap } from 'rxjs';
import { CodeService } from '../services/code.service';
import { LandingStateService } from '../services/landing-state.service';
import { navigationPerf, printSingle } from '../utilities/perf-utils';

@Component({
  selector: 'app-loading',
  templateUrl: './loading.component.html',
  styleUrls: ['./loading.component.css']
})
export class LoadingComponent implements OnInit, OnDestroy {

  progress = 0;
  private sub?: Subscription;

  constructor(private code: CodeService,
    private router: Router,
    private landingState: LandingStateService) { }

  ngOnInit(): void {
    this.sub = this.code.progress$.subscribe(p => this.progress = p);
    this.code.ready$.pipe(
      filter(Boolean),
      take(1),
      tap(() => {
        navigationPerf.mark('loaded_workspace');
        navigationPerf.measure(
          'time_to_workspace',
          'get_started_click',
          'loaded_workspace'
        );
        printSingle('navigation', 'time_to_workspace');
        this.landingState.markVisited()
      }
      )
    ).subscribe(() => {
      this.router.navigate(['/workspace']);
    });
  }


  ngOnDestroy(): void { this.sub?.unsubscribe(); }
}
