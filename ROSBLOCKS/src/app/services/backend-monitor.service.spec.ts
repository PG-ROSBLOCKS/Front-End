import { TestBed } from '@angular/core/testing';

import { BackendMonitorService } from './backend-monitor.service';

describe('BackendMonitorService', () => {
  let service: BackendMonitorService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(BackendMonitorService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
