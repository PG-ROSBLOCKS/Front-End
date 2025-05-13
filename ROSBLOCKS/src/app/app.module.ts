import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { FormsModule } from '@angular/forms';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { WorkspaceComponent } from './workspace/workspace.component';
import { HeaderComponent } from './shared/components/header/header.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { SafePipe } from './safe.pipe';
import { HTTP_INTERCEPTORS, HttpClientModule } from '@angular/common/http';
import { ConfirmComponent } from './shared/components/confirm/confirm.component';
import { AlertComponent } from './shared/components/alert/alert.component';
import { ResizerDirective } from './workspace/resizer.directive';
import { ConsoleOutputComponent } from './console-output/console-output.component';
import { SuccessComponent } from './shared/components/success/success.component';
import { LandingComponent } from './landing/landing.component';
import { RouteReuseStrategy } from '@angular/router';
import { SimpleReuseStrategy } from './landing/reuse-strategy';
import { BackendMonitorService } from './services/backend-monitor.service';

@NgModule({
  declarations: [
    AppComponent,
    WorkspaceComponent,
    HeaderComponent,
    RosblocksHelpComponent,
    SafePipe,
    ConfirmComponent,
    AlertComponent,
    ResizerDirective,
    ConsoleOutputComponent,
    SuccessComponent,
    LandingComponent,
  ],
  imports: [
    BrowserModule,
    FormsModule,
    AppRoutingModule,
    HttpClientModule,
  ],
  providers: [
    { provide: RouteReuseStrategy, 
      useClass: SimpleReuseStrategy 
    }, 
    { provide: HTTP_INTERCEPTORS,
    useClass: BackendMonitorService,
    multi: true
  }],
  bootstrap: [AppComponent]
})
export class AppModule { }
