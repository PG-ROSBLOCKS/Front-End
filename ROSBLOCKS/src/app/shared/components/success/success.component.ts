import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-success',
  templateUrl: './success.component.html',
  styleUrls: ['./success.component.css']
})
export class SuccessComponent {
  @Input() message: string = 'Alerta';
  @Output() ok = new EventEmitter<boolean>();
    
  Done(): void {
    this.ok.emit(true);
  }
}
