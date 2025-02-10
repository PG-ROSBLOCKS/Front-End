import { HttpClient } from '@angular/common/http';
import { Component } from '@angular/core';

@Component({
  selector: 'app-console',
  templateUrl: './console.component.html',
})
export class ConsoleComponent {
  output: string = '';
  
  constructor(private http: HttpClient) {}

  executeCode(code: string) {
    this.http.post<{ output: string; error: string }>('http://localhost:8000/execute', { code })
      .subscribe(response => {
        this.output = response.output || response.error;
      });
  }
}
