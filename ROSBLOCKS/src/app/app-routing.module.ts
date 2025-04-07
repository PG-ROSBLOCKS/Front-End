import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { WorkspaceComponent } from './workspace/workspace.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { OauthCallbackComponent } from './shared/oauth-callback/oauth-callback.component';
import { AuthGuard } from './auth.guard';

const routes: Routes = [
  {
    path: '',
    component: WorkspaceComponent,
    canActivate: [AuthGuard], // protege la ruta principal
  },
  {
    path: 'oauth-callback', 
    component: OauthCallbackComponent,
  },
  { path: '**', redirectTo: '' },
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
