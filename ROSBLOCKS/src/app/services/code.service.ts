import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { WebSocketSubject, webSocket } from 'rxjs/webSocket';
import { BehaviorSubject, Observable, tap, map } from 'rxjs';
import { safeUUID } from '../utilities/sanitizer-tools';

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

  private readySubject    = new BehaviorSubject<boolean>(false);
  private progressSubject = new BehaviorSubject<number>(0);

  ready$    = this.readySubject.asObservable();
  progress$ = this.progressSubject.asObservable();   // 0 – 100 %

  private API_URL = 'http://localhost:8000';
  private uuid: string = '';
  private DOMAIN = 'https://api.rosblocks.com.co';


  constructor(private http: HttpClient) {
    this.wsSubject = undefined;
    this.uuid = localStorage.getItem('uuid') ?? safeUUID();
    localStorage.setItem('uuid', this.uuid);
  
    this.pollForIp(this.uuid);
  }
  
  private async pollForIp(uuid: string) {
    try {
      const res = await fetch(`${this.DOMAIN}/api/get-ip/${uuid}`);
      const data = await res.json();
  
      if (data.status === "ready") {
        this.API_URL = `${this.DOMAIN}/session/${uuid}/app`;
        this.readySubject.next(true);
      } else {
        this.progressSubject.next((this.progressSubject.value + 5) % 100);
        setTimeout(() => this.pollForIp(uuid), 5000);
      }
    } catch (err) {
      setTimeout(() => this.pollForIp(uuid), 5000);
    }
  }
  
  uploadCode(fileName: string, code: string, type: string): Observable<any> {

    const payload = { file_name: fileName, code: code, type: type };
    console.log(payload);
    
    return this.http.post(`${this.API_URL}/upload/`, payload, {
      headers: { 'Content-Type': 'application/json' },
    });
  }

  executeCode(fileName: string): Observable<any> {
    return this.http.get(`${this.API_URL}/execution/execute/${fileName}`);
  }

  connectToWebSocket(sessionId: string): WebSocketSubject<any> {
    this.wsSubject = webSocket(
      `wss://api.rosblocks.com.co/session/${this.uuid}/app/execution/ws/${sessionId}`
    );
    return this.wsSubject;
  }
  

  sendMessage(message: string): void {
    if (this.wsSubject) {
      this.wsSubject.next(message);
    }
  }

  killExecution(session_id: string): void {
    console.log(session_id);
    this.http.get(`${this.API_URL}/execution/kill/${session_id}`, { responseType: 'json' })
      .subscribe({
        next: (response) => {
          console.log(session_id);
          
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
    this.http.delete(`${this.API_URL}/execution/cleanup/${fileName}`, { responseType: 'json' })
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
    this.http.get(`${this.API_URL}/export/`, { responseType: 'blob' }).subscribe(response => {
      const blob = new Blob([response], { type: 'application/gzip' });

      // Create download link with default name
      const link = document.createElement('a');
      link.href = window.URL.createObjectURL(blob);
      link.download = 'ros2_ws.tar.gz'; // default name
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);

    }, error => {
      console.error('Error deleting project:', error);
    });
  }

  checkSrvFiles(): Observable<{ exists: boolean, files: string[] }> {
    return this.http.get<{ exists: boolean, files: string[] }>(`${this.API_URL}/srvfiles`)
      .pipe(
      tap(response => console.log('checkSrvFiles returns:', response))
    );
  }

  checkMsgFiles(): Observable<{ exists: boolean, files: string[] }> {
    return this.http.get<{ exists: boolean, files: string[] }>(`${this.API_URL}/msgfiles`)
      .pipe(
      tap(response => console.log('checkMsgFiles returns:', response))
    );
  }

  //Funtion to delete a .srv or a .msg of the project
  deleteInterfaceFile(fileType: 'srv' | 'msg', fileName: string): Observable<any> {
    console.log('El endpoint es:', `${this.API_URL}/delete/interfaces/${fileType}/${fileName}`);
    return this.http.delete(`${this.API_URL}/delete/interfaces/${fileType}/${fileName}/`);
  }

  vncTurtlesim(): string {
    return `${this.DOMAIN}/session/${this.uuid}/vnc/vnc_lite.html`;
  }  

  vncTurtlesimReset(): string {
    return `${this.API_URL}/reset/`;
  }

  seeTopics(): Observable<string[]> {
    interface ViewTopicsResponse {
      message: string;
      output: string;
    }
    console.log('El endpoint es:', `${this.API_URL}/ros_commands/view_topics`);
    return this.http.get<ViewTopicsResponse>(`${this.API_URL}/ros_commands/view_topics`).pipe(
      map((response: ViewTopicsResponse): string[] => {
      return response.output.trim().split('\n').filter((topic: string) => topic.trim() !== '');
      })
    );
  }
  
}
