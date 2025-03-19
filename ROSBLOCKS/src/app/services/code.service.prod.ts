import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { WebSocketSubject, webSocket } from 'rxjs/webSocket';
import { BehaviorSubject, Observable, tap } from 'rxjs';
import { environment } from '../environments/environment';

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
  private API_URL = environment.API_URL;;
  private API_URL_NO_PORT = 'http://localhost:';
  private podUrl: string | null = null;

  constructor(private http: HttpClient) {
    this.wsSubject = undefined;
    this.initializePodUrl();
  }

  private initializePodUrl(): void {
    const sessionId = this.generateSessionId();
    this.http.get<{ pod_url: string }>(`${this.API_URL}/get-user-pod/${sessionId}`)
      .subscribe(response => {
        this.podUrl = response.pod_url;
      }, error => {
        console.error('Error obteniendo el pod:', error);
      });
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
    return this.http.post(`${this.podUrl}/upload/`, payload, {
      headers: { 'Content-Type': 'application/json' },
    });
  }

  executeCode(fileName: string): Observable<any> {
    return this.http.get(`${this.podUrl}/execution/execute/${fileName}`);
  }

  connectToWebSocket(sessionId: string): WebSocketSubject<any> {
    if (this.podUrl) {
      this.wsSubject = webSocket(`${this.podUrl.replace('http', 'ws')}/execution/ws/${sessionId}`);
    } else {
      throw new Error('Pod URL is not initialized.');
    }
    return this.wsSubject;
  }

  sendMessage(message: string): void {
    if (this.wsSubject) {
      this.wsSubject.next(message);
    }
  }

  killExecution(session_id: string): void {
    this.http.get(`${this.podUrl}/execution/kill/${session_id}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log('Sesión successfully deleted:', response);
          this.closeConnection();
        },
        error: (error) => console.error('Error killing the session:', error)
      });
  }

  closeConnection(): void {
    if (this.wsSubject) {
      this.wsSubject.complete();
      this.wsSubject = undefined;
    }
  }

  deleteFile(fileName: string): void {
    this.http.delete(`${this.podUrl}/execution/cleanup/${fileName}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log(`Archivo ${fileName} successfully deleted:`, response);
          this.closeConnection();
        },
        error: (error) => console.error(`Error deleting ${fileName}:`, error)
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
    this.http.get(`${this.podUrl}/export/`, { responseType: 'blob' }).subscribe(response => {
      const blob = new Blob([response], { type: 'application/gzip' });
      const link = document.createElement('a');
      link.href = window.URL.createObjectURL(blob);
      link.download = 'ros2_ws.tar.gz';
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }, error => {
      console.error('Error deleting proyect:', error);
    });
  }

  checkSrvFiles(): Observable<{ exists: boolean, files: string[] }> {
    return this.http.get<{ exists: boolean, files: string[] }>(`${this.podUrl}/srvfiles`)
      .pipe(
        tap(response => console.log('checkSrvFiles returns:', response))
      );
  }

  deleteInterfaceFile(fileType: 'srv' | 'msg', fileName: string): Observable<any> {
    console.log('El endpoint es:', `${this.podUrl}/delete/interfaces/${fileType}/${fileName}`);
    return this.http.delete(`${this.podUrl}/delete/interfaces/${fileType}/${fileName}/`);
  }

  vncTurtlesim(): string {
    return `${this.podUrl}/vnc_auto.html`;
  }

  vncTurtlesimReset(): string {
    return `${this.podUrl}/reset/`;
  }
}
