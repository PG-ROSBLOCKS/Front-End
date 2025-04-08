import { Component, Input, AfterViewChecked, ElementRef, ViewChild } from '@angular/core';

@Component({
  selector: 'app-console-output',
  templateUrl: './console-output.component.html',
  styleUrls: ['./console-output.component.css']
})
export class ConsoleOutputComponent implements AfterViewChecked {
  // Receive the output array of lines from the parent
  @Input() output: string[] = [];
  // Controls if auto-scroll is enables
  @Input() autoScrollEnabled: boolean = true;
  @ViewChild('consoleContainer') consoleContainer!: ElementRef<HTMLDivElement>;

  private previousScrollHeight = 0;
  private userScrolled = false;

  ngAfterViewInit(): void {
    if (this.consoleContainer) {
      this.consoleContainer.nativeElement.addEventListener('scroll', () => {
        const el = this.consoleContainer.nativeElement;
        const isAtBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 10;
        this.userScrolled = !isAtBottom;
      });
    }
  }

  ngAfterViewChecked(): void {
    if (!this.consoleContainer) return;

    const el = this.consoleContainer.nativeElement;
    const currentScrollHeight = el.scrollHeight;

    const contentGrew = currentScrollHeight > this.previousScrollHeight;

    if (this.autoScrollEnabled && contentGrew && !this.userScrolled) {
      el.scrollTop = currentScrollHeight;
    }

    this.previousScrollHeight = currentScrollHeight;
  }
  
  
}
