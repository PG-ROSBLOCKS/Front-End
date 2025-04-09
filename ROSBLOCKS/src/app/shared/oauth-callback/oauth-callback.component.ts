import { Component, OnInit } from '@angular/core';
import { ActivatedRoute, Router } from '@angular/router';
import { HttpClient, HttpParams } from '@angular/common/http';
import { environment } from '../../../environments/environment.development';

@Component({
  selector: 'app-oauth-callback',
  template: `<p>Authenticating...</p>`
})
export class OauthCallbackComponent implements OnInit {
  constructor(
    private route: ActivatedRoute,
    private http: HttpClient,
    private router: Router
  ) {}
  token: string = '';

  ngOnInit() {
    this.route.queryParams.subscribe(params => {
      const code = params['code'];
      if (code) {
        const body = new HttpParams().set('code', code);
        this.http.post<any>(`http://${environment.backendFastAPIAddress}/services/fastapi/get_token`, body)
          .subscribe({
            next: (response) => {
              console.log('Response:', response);
              this.token = response.access_token;
              localStorage.setItem('access_token', this.token);
              console.log('Token:', this.token);
  
              // Opcional: redirige al home después de unos segundos
              setTimeout(() => this.router.navigate(['/']), 2000);
            },
            error: (err) => {
              console.error('Error al obtener token:', err);
            }
          });
      }
    });
  }
}
