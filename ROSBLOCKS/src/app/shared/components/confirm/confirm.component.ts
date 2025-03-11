import { Component, Input, Output, EventEmitter } from '@angular/core';

@Component({
  selector: 'app-confirm',
  templateUrl: './confirm.component.html',
  styleUrls: ['./confirm.component.css']
})
export class ConfirmComponent {
  @Input() message: string = '¿Estás seguro?';
  
  // Emits true if confirmed, false if canceled
  @Output() confirm = new EventEmitter<boolean>();

  confirmar(): void {
    this.confirm.emit(true);
  }

  cancelar(): void {
    this.confirm.emit(false);
  }
}
