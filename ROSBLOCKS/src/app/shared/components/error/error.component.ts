import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-error',
  templateUrl: './error.component.html',
  styleUrls: ['./error.component.css']
})
export class ErrorComponent {
  @Input() message: string = 'Error';
  @Output() ok = new EventEmitter<boolean>();
    
  Done(): void {
    this.ok.emit(true);
  }
}
