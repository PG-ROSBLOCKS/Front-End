import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { WorkspaceComponent } from './workspace/workspace.component';
import { ErrorComponent } from './shared/components/error/error.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { AlertComponent } from './shared/components/alert/alert.component';

const routes: Routes = [
  {path: 'help', component: RosblocksHelpComponent},
  {path: '**', component: WorkspaceComponent}
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
