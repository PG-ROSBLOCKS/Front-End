import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { FormsModule } from '@angular/forms';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { WorkspaceComponent } from './workspace/workspace.component';
import { HeaderComponent } from './shared/components/header/header.component';
import { ErrorComponent } from './shared/components/error/error.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { SafePipe } from './safe.pipe';
import { HttpClientModule } from '@angular/common/http';
import { ConfirmComponent } from './shared/components/confirm/confirm.component';
import { AlertComponent } from './shared/components/alert/alert.component';
import { ResizerDirective } from './workspace/resizer.directive';
import { ConsoleOutputComponent } from './console-output/console-output.component';
import { SuccessComponent } from './shared/components/success/success.component';

@NgModule({
  declarations: [
    AppComponent,
    WorkspaceComponent,
    HeaderComponent,
    ErrorComponent,
    RosblocksHelpComponent,
    SafePipe,
    ConfirmComponent,
    AlertComponent,
    ResizerDirective,
    ConsoleOutputComponent,
    SuccessComponent,
  ],
  imports: [
    BrowserModule,
    FormsModule,
    AppRoutingModule,
    HttpClientModule,
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
