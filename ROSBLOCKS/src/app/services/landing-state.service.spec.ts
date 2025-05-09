import { TestBed } from '@angular/core/testing';

import { LandingStateService } from './landing-state.service';

describe('LandingStateService', () => {
  let service: LandingStateService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(LandingStateService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
