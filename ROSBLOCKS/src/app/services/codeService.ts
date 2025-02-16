import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { WebSocketSubject, webSocket } from 'rxjs/webSocket';
import { Observable, BehaviorSubject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class CodeService {
  private wsSubject: WebSocketSubject<any> | undefined;
  private outputSubject = new BehaviorSubject<string>('');

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
    //return this.http.get(`http://localhost:8000/execute/minimal_publisher.py`);
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

  closeConnection() {
    if (this.wsSubject) {
      this.wsSubject.complete();
    }
  }
  // Método para obtener el observable del string
  getOutputObservable(): Observable<string> {
    return this.outputSubject.asObservable();
  }

  // Método para actualizar el string
  updateOutput(newOutput: string) {
    this.outputSubject.next(newOutput);
    console.log(newOutput);
  }
}
