import { Component, Input } from '@angular/core';

@Component({
  selector: 'app-error-msg',
  templateUrl: './error-msg.component.html',
  styleUrls: ['./error-msg.component.css']
})
export class ErrorMsgComponent {
  @Input() message: string = 'Mensaje de prueba';
  isVisible: boolean = true;

  closeMessage() {
    this.isVisible = false;
  }
}
