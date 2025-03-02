import { Directive, ElementRef, Input, Renderer2, OnInit, OnDestroy } from '@angular/core';

@Directive({
  selector: '[appResizer]'
})
export class ResizerDirective implements OnInit, OnDestroy {
  @Input() leftContainer!: HTMLElement;
  @Input() rightContainer!: HTMLElement;
  @Input() containerSelector: string = '#workspace-container';

  private mouseMoveHandler!: (event: MouseEvent) => void;
  private mouseUpHandler!: (event: MouseEvent) => void;
  private unlistenMouseDown: (() => void) | null = null;

  constructor(private el: ElementRef, private renderer: Renderer2) {}

  ngOnInit(): void {
    // Escuchamos el mousedown en el elemento al que se aplica la directiva
    this.unlistenMouseDown = this.renderer.listen(this.el.nativeElement, 'mousedown', this.onMouseDown.bind(this));
  }

  onMouseDown(event: MouseEvent): void {
    this.mouseMoveHandler = this.onMouseMove.bind(this);
    this.mouseUpHandler = this.onMouseUp.bind(this);
    document.addEventListener('mousemove', this.mouseMoveHandler);
    document.addEventListener('mouseup', this.mouseUpHandler);
  }

  onMouseMove(event: MouseEvent): void {
    const newWidth = event.clientX;
    const container = document.querySelector(this.containerSelector) as HTMLElement;
    if (container) {
      const containerWidth = container.offsetWidth;
      if (newWidth > 100 && newWidth < containerWidth * 0.8) {
        this.leftContainer.style.width = `${newWidth}px`;
        this.rightContainer.style.flex = '1';
      }
    }
  }

  onMouseUp(event: MouseEvent): void {
    document.removeEventListener('mousemove', this.mouseMoveHandler);
    document.removeEventListener('mouseup', this.mouseUpHandler);
  }

  ngOnDestroy(): void {
    if (this.unlistenMouseDown) {
      this.unlistenMouseDown();
    }
    document.removeEventListener('mousemove', this.mouseMoveHandler);
    document.removeEventListener('mouseup', this.mouseUpHandler);
  }
}
