import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { FormsModule } from '@angular/forms';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { WorkspaceComponent } from './workspace/workspace.component';
import { HeaderComponent } from './shared/components/header/header.component';
import { ErrorMsgComponent } from './shared/components/error-msg/error-msg.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { SafePipe } from './safe.pipe';
import { HttpClientModule } from '@angular/common/http';
import { ConfirmComponent } from './shared/components/confirm/confirm.component';

@NgModule({
  declarations: [
    AppComponent,
    WorkspaceComponent,
    HeaderComponent,
    ErrorMsgComponent,
    RosblocksHelpComponent,
    SafePipe,
    ConfirmComponent,
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
