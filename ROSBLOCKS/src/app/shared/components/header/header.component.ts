import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { CodeService } from 'src/app/services/codeService';

@Component({
  selector: 'app-header',
  templateUrl: './header.component.html',
  styleUrls: ['./header.component.css']
})
export class HeaderComponent {

  showExport: boolean = true;

  constructor(private router: Router, private service: CodeService) {}

  ngOnInit() {
    this.router.events.subscribe(() => {
      this.showExport = this.router.url !== '/help';
    });
  }

  reloadPage() {
    window.location.reload();
  }

  export() {
    this.service.exportProject();
  }
}