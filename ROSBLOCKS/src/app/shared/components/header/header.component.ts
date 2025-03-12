import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { Subscription } from 'rxjs';
import { CodeService } from 'src/app/services/code.service';
import { AlertService } from '../alert/alert.service';
import { workspaceComments } from 'blockly/core/serialization';
import { WorkspaceComponent } from 'src/app/workspace/workspace.component';

@Component({
  selector: 'app-header',
  templateUrl: './header.component.html',
  styleUrls: ['./header.component.css']
})
export class HeaderComponent {

  workspaceChanged: boolean = false;
  noTabs: boolean = true;
  noBlocks: boolean = true;
  subscription!: Subscription;

  showExport: boolean = true;

  constructor(private router: Router, 
    private service: CodeService, 
    private alertService: AlertService,
    private workspace: WorkspaceComponent
  ) { }

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
    this.subscription = this.service.noBlocks$.subscribe((noBlocks: boolean) => {
      this.noBlocks = noBlocks;
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

  safe() {
    this.workspace.saveToLocalStorage();
  }

  load() {
    this.workspace.loadFromLocalStorage();
  }

  async export() {
    if (this.noTabs == true || this.noBlocks == true) {
      const result = await this.alertService.showAlert("El proyecto está vacío. Agrega nodos y/o bloques antes de exportar")
    }
    else {
      if (this.workspaceChanged == false) {
        this.service.exportProject();
      } else
          await this.alertService.showAlert("Realizaste un cambio en el nodo, primero debes ejecutar los cambios antes de exportar")
    }
  }
}