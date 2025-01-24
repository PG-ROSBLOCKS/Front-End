import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BlocklyComponent } from './blockly.component';

describe('BlocklyComponent', () => {
  let component: BlocklyComponent;
  let fixture: ComponentFixture<BlocklyComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [BlocklyComponent]
    });
    fixture = TestBed.createComponent(BlocklyComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
