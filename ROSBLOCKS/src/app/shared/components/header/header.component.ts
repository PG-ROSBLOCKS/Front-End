import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { Subscription } from 'rxjs';
import { CodeService } from 'src/app/services/code.service';
import { AlertService } from '../alert/alert.service';
import { workspaceComments } from 'blockly/core/serialization';
import { WorkspaceComponent } from 'src/app/workspace/workspace.component';
import { UserService } from '../../user.service';

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
  isLoggedIn = false;
  showExport: boolean = true;
  userInfo: any = null;

  constructor(private router: Router, 
    private service: CodeService, 
    private alertService: AlertService,
    private workspace: WorkspaceComponent,
    private userService: UserService
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
    const token = localStorage.getItem('access_token');
    this.isLoggedIn = !!token;

    if(this.isLoggedIn) {
      this.userService.getUserInfo().subscribe(
        (response) => {
          this.userInfo = response;
        },
        (error) => {
          console.error('Error fetching user info:', error);
        }
      );
    }
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
    this.workspace.saveToFile();
  }

  load(event: Event) {
    console.log(2);
    
    this.workspace.loadFromFile(event);
  }

  async export() {
    if (this.noTabs == true || this.noBlocks == true) {
      const result = await this.alertService.showAlert("The project is empty. Add nodes and/or blocks before exporting");
        } else {
      if (this.workspaceChanged == false) {
        this.service.exportProject();
      } else {
        await this.alertService.showAlert("You made a change to the node, you must execute the changes first before exporting");
      }
        }
    }

    startOAuthLogin() {
      const clientId = 'service-fastapi';
      const redirectUri = encodeURIComponent('http://localhost:4200/oauth-callback');
      const state = crypto.randomUUID();
    
      window.location.href = `http://34.58.80.154/hub/api/oauth2/authorize?client_id=${clientId}&redirect_uri=${redirectUri}&response_type=code&state=${state}`;
    }
    
  }