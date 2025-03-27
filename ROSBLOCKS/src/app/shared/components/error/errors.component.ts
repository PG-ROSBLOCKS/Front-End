import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'errors-alert',
  templateUrl: './errors.component.html',
  styleUrls: ['./errors.component.css']
})
export class ErrorsComponent {
  @Input() message: string = 'Errors';
  @Input() showCancel: boolean = false;
  @Output() ok = new EventEmitter<boolean>();
  
  Done(): void {
    this.ok.emit(true);
  }
  
  Cancel(): void {
    this.ok.emit(false);
  }
}