import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { FormsModule } from '@angular/forms';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { WorkspaceComponent } from './workspace/workspace.component';
import { HeaderComponent } from './shared/components/header/header.component';
import { ErrorMsgComponent } from './shared/components/error-msg/error-msg.component';

@NgModule({
  declarations: [
    AppComponent,
    WorkspaceComponent,
    HeaderComponent,
    ErrorMsgComponent
  ],
  imports: [
    BrowserModule,
    FormsModule,
    AppRoutingModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
