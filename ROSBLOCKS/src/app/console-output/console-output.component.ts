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

  ngAfterViewChecked(): void {
    if (this.autoScrollEnabled && this.consoleContainer) {
      this.consoleContainer.nativeElement.scrollTop = this.consoleContainer.nativeElement.scrollHeight;
    }
  }
}
