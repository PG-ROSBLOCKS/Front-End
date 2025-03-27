// message.service.ts
import { Injectable } from '@angular/core';
import { Subject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class MessageService {
  private messageSource = new Subject<any>();
  message$ = this.messageSource.asObservable();

  sendMessage(message: any) {
    this.messageSource.next(message);
  }
}
