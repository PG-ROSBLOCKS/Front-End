import { TestBed } from '@angular/core/testing';
import { CanActivateFn } from '@angular/router';

import { landingVisitedGuard } from './landing-visited.guard';

describe('landingVisitedGuard', () => {
  const executeGuard: CanActivateFn = (...guardParameters) => 
      TestBed.runInInjectionContext(() => landingVisitedGuard(...guardParameters));

  beforeEach(() => {
    TestBed.configureTestingModule({});
  });

  it('should be created', () => {
    expect(executeGuard).toBeTruthy();
  });
});
