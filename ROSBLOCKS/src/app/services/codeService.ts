import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { WebSocketSubject, webSocket } from 'rxjs/webSocket';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class CodeService {
  private wsSubject: WebSocketSubject<any> | undefined;
  private API_URL = 'http://localhost:8000';

  constructor(private http: HttpClient) {
    this.wsSubject = undefined;
  }

  uploadCode(fileName: string, code: string): Observable<any> {
    const payload = { file_name: fileName, code: code };
    return this.http.post(`${this.API_URL}/upload/`, payload, {
      headers: { 'Content-Type': 'application/json' }, 
    });
  }

  executeCode(fileName: string): Observable<any> {
    return this.http.get(`${this.API_URL}/execute/${fileName}`);
  }
  
  connectToWebSocket(sessionId: string): WebSocketSubject<any> {
    this.wsSubject = webSocket(`${this.API_URL.replace('http', 'ws')}/ws/${sessionId}`);
    return this.wsSubject;
  }

  sendMessage(message: string): void {
    if (this.wsSubject) {
      this.wsSubject.next(message);
    }
  }

  killExecution(session_id: string): void {
    this.http.get(`${this.API_URL}/kill/${session_id}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log('Sesión eliminada con éxito:', response);
          this.closeConnection(); 
        },
        error: (error) => console.error('Error al matar la sesión:', error)
      });
  }
  closeConnection(): void {
    if (this.wsSubject) {
      this.wsSubject.complete();
      this.wsSubject = undefined;
    }
  }

  deleteFile(fileName: string): void {
    this.http.delete(`${this.API_URL}/cleanup/${fileName}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log(`Archivo ${fileName} eliminado con éxito:`, response);
          this.closeConnection(); 
        },
        error: (error) => console.error(`Error al eliminar ${fileName}:`, error)
      });
  }
}
