import { Component, OnDestroy, OnInit } from '@angular/core';
import { Router } from '@angular/router';
import { filter, Subscription, take } from 'rxjs';
import { CodeService } from '../services/code.service';

@Component({
  selector: 'app-loading',
  templateUrl: './loading.component.html',
  styleUrls: ['./loading.component.css']
})
export class LoadingComponent implements OnInit, OnDestroy {

  progress = 0;
  private sub?: Subscription;

  constructor(private code: CodeService,
              private router: Router) {}

  ngOnInit(): void {
    /*progress bar */
    this.sub = this.code.progress$.subscribe(p => this.progress = p);

    this.code.ready$.pipe(
      filter(Boolean), take(1)
    ).subscribe(() => this.router.navigate(['/workspace']));
  }

  ngOnDestroy(): void { this.sub?.unsubscribe(); }
}
