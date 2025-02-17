import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { WebSocketSubject, webSocket } from 'rxjs/webSocket';
import { Observable, BehaviorSubject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class CodeService {
  private wsSubject: WebSocketSubject<any> | undefined;

  constructor(private http: HttpClient) {
    this.wsSubject = undefined;
  }

  uploadCode(fileName: string, code: string) {
    const payload = {
      file_name: fileName,
      code: code,
    };
  
    return this.http.post('http://localhost:8000/upload/', payload, {
      headers: { 'Content-Type': 'application/json' }, 
    });
  }

  executeCode(fileName: string): Observable<any> {
    return this.http.get(`http://localhost:8000/execute/${fileName}`);
  }
  
  connectToWebSocket(sessionId: string): WebSocketSubject<any> {
    this.wsSubject = webSocket(`ws://localhost:8000/ws/${sessionId}`);
    return this.wsSubject;
  }

  sendMessage(message: string) {
    if (this.wsSubject) {
      this.wsSubject.next(message);
    }
  }

  killExecution(session_id: string) {
    this.http.get(`http://localhost:8000/kill/${session_id}`, { responseType: 'json' })
      .subscribe({
        next: (response) => console.log('Sesión eliminada con éxito:', response),
        error: (error) => console.error('Error en la solicitud:', error)
      });
}


  closeConnection() {
    if (this.wsSubject) {
      this.wsSubject.complete();
    }
  }
}
