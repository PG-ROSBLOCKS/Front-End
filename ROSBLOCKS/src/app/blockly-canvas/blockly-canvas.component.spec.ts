import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BlocklyCanvasComponent } from './blockly-canvas.component';

describe('BlocklyCanvasComponent', () => {
  let component: BlocklyCanvasComponent;
  let fixture: ComponentFixture<BlocklyCanvasComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [BlocklyCanvasComponent]
    });
    fixture = TestBed.createComponent(BlocklyCanvasComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
