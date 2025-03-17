import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { WebSocketSubject, webSocket } from 'rxjs/webSocket';
import { BehaviorSubject, Observable, tap } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class CodeService {
  private wsSubject: WebSocketSubject<any> | undefined;
  private workspaceChangedSubject = new BehaviorSubject<boolean>(false);
  private noTabsSubject = new BehaviorSubject<boolean>(true);
  private noBlocksSubject = new BehaviorSubject<boolean>(true);
  workspaceChanged$ = this.workspaceChangedSubject.asObservable();
  noTabs$ = this.noTabsSubject.asObservable();
  noBlocks$ = this.noBlocksSubject.asObservable();
  private API_URL = process.env['API_URL'] || 'https://tu-fastapi-aws.com';
  private API_URL_NO_PORT = this.API_URL.replace(/:\d+$/, '');

  constructor(private http: HttpClient) {
    this.wsSubject = undefined;
  }

  private generateSessionId(): string {
    let sessionId = localStorage.getItem('session_id');
    if (!sessionId) {
      sessionId = Math.random().toString(36).substring(2, 15);
      localStorage.setItem('session_id', sessionId);
    }
    return sessionId;
  }

  uploadCode(fileName: string, code: string, type: string): Observable<any> {
    const payload = { file_name: fileName, code: code, type: type };
    return this.http.post(`${this.API_URL}/upload/`, payload, {
      headers: { 'Content-Type': 'application/json' },
    });
  }

  executeCode(fileName: string): Observable<any> {
    return this.http.get(`${this.API_URL}/execution/execute/${fileName}`);
  }

  connectToWebSocket(): WebSocketSubject<any> {
    const sessionId = this.generateSessionId();
    this.wsSubject = webSocket(`${this.API_URL.replace('http', 'ws')}/execution/ws/${sessionId}`);
    return this.wsSubject;
  }

  sendMessage(message: string): void {
    if (this.wsSubject) {
      this.wsSubject.next(message);
    }
  }

  killExecution(): void {
    const sessionId = localStorage.getItem('session_id');
    if (!sessionId) return;
    this.http.get(`${this.API_URL}/execution/kill/${sessionId}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log('Sesión eliminada:', response);
          this.closeConnection();
        },
        error: (error) => console.error('Error eliminando la sesión:', error)
      });
  }

  closeConnection(): void {
    if (this.wsSubject) {
      this.wsSubject.complete();
      this.wsSubject = undefined;
    }
  }

  deleteFile(fileName: string): void {
    this.http.delete(`${this.API_URL}/execution/cleanup/${fileName}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log(`Archivo ${fileName} eliminado:`, response);
          this.closeConnection();
        },
        error: (error) => console.error(`Error eliminando ${fileName}:`, error)
      });
  }

  setWorkspaceChanged(flag: boolean) {
    this.workspaceChangedSubject.next(flag);
  }

  setNoTabs(flag: boolean) {
    this.noTabsSubject.next(flag);
  }

  setNoBlocks(flag: boolean) {
    this.noBlocksSubject.next(flag);
  }

  exportProject(): void {
    this.http.get(`${this.API_URL}/export/`, { responseType: 'blob' }).subscribe(response => {
      const blob = new Blob([response], { type: 'application/gzip' });

      // Crear enlace de descarga
      const link = document.createElement('a');
      link.href = window.URL.createObjectURL(blob);
      link.download = 'ros2_ws.tar.gz';
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);

    }, error => {
      console.error('Error exportando proyecto:', error);
    });
  }

  checkSrvFiles(): Observable<{ exists: boolean, files: string[] }> {
    return this.http.get<{ exists: boolean, files: string[] }>(`${this.API_URL}/srvfiles`)
      .pipe(
        tap(response => console.log('checkSrvFiles returns:', response))
      );
  }

  deleteInterfaceFile(fileType: 'srv' | 'msg', fileName: string): Observable<any> {
    console.log('El endpoint es:', `${this.API_URL}/delete/interfaces/${fileType}/${fileName}`);
    return this.http.delete(`${this.API_URL}/delete/interfaces/${fileType}/${fileName}/`);
  }

  vncTurtlesim(): string {
    return `${this.API_URL_NO_PORT}8080/vnc_auto.html`;
  }

  vncTurtlesimReset(): string {
    return `${this.API_URL}/reset/`;
  }
}
