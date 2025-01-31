import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RosblocksHelpComponent } from './rosblocks-help.component';

describe('RosblocksHelpComponent', () => {
  let component: RosblocksHelpComponent;
  let fixture: ComponentFixture<RosblocksHelpComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [RosblocksHelpComponent]
    });
    fixture = TestBed.createComponent(RosblocksHelpComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
