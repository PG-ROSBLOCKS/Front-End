import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-alert',
  templateUrl: './alert.component.html',
  styleUrls: ['./alert.component.css']
})
export class AlertComponent {
  @Input() message: string = 'Alerta';
  @Output() ok = new EventEmitter<boolean>();
  
  Done(): void {
    this.ok.emit(true);
  }
}
