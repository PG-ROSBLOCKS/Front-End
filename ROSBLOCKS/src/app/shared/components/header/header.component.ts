import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { Subscription } from 'rxjs';
import { CodeService } from 'src/app/services/codeService';

@Component({
  selector: 'app-header',
  templateUrl: './header.component.html',
  styleUrls: ['./header.component.css']
})
export class HeaderComponent {

  workspaceChanged: boolean = false;
  noTabs: boolean = true;
  subscription!: Subscription;

  showExport: boolean = true;

  constructor(private router: Router, private service: CodeService) { }

  ngOnInit() {
    this.router.events.subscribe(() => {
      this.showExport = this.router.url !== '/help';
    });
    this.subscription = this.service.workspaceChanged$.subscribe((changed: boolean) => {
      this.workspaceChanged = changed;
    });
    this.subscription = this.service.noTabs$.subscribe((noTabs: boolean) => {
      this.noTabs = noTabs;
    });
  }

  ngOnDestroy() {
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }

  reloadPage() {
    window.location.reload();
  }

  export() {
    if (this.noTabs == true) {
      alert("El proyecto está vacío. Agrega bloques antes de exportar")
    }
    else {
      if (this.workspaceChanged == false) {
        this.service.exportProject();
      } else
        alert("Realizaste un cambio en el nodo, primero debes ejecutar los cambios antes de exportar")
    }
  }
}