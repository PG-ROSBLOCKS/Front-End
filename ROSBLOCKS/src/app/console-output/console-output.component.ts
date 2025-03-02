import { Component, Input, AfterViewChecked, ElementRef, ViewChild } from '@angular/core';

@Component({
  selector: 'app-console-output',
  templateUrl: './console-output.component.html',
  styleUrls: ['./console-output.component.css']
})
export class ConsoleOutputComponent implements AfterViewChecked {
  // Recibe el arreglo de líneas de salida desde el padre
  @Input() output: string[] = [];
  // Controla si el auto-scroll está activado
  @Input() autoScrollEnabled: boolean = true;
  @ViewChild('consoleContainer') consoleContainer!: ElementRef<HTMLDivElement>;

  ngAfterViewChecked(): void {
    if (this.autoScrollEnabled && this.consoleContainer) {
      this.consoleContainer.nativeElement.scrollTop = this.consoleContainer.nativeElement.scrollHeight;
    }
  }
}
