import { Component, Input, Output, EventEmitter } from '@angular/core';

@Component({
  selector: 'app-confirm',
  templateUrl: './confirm.component.html',
  styleUrls: ['./confirm.component.css']
})
export class ConfirmComponent {
  @Input() message: string = '¿Estás seguro?';
  
  // Emite true si se confirma, false si se cancela
  @Output() confirm = new EventEmitter<boolean>();

  confirmar(): void {
    this.confirm.emit(true);
  }

  cancelar(): void {
    this.confirm.emit(false);
  }
}
