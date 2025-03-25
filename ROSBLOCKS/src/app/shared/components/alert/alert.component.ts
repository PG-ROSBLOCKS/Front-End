import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-alert',
  templateUrl: './alert.component.html',
  styleUrls: ['./alert.component.css']
})
export class AlertComponent {
  @Input() message: string = 'Alerta';
  @Input() showCancel: boolean = false;
  @Output() ok = new EventEmitter<boolean>();
  
  Done(): void {
    this.ok.emit(true);
  }
  
  Cancel(): void {
    this.ok.emit(false);
  }
}