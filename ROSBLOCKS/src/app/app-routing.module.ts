import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { WorkspaceComponent } from './workspace/workspace.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';

const routes: Routes = [
  {path: 'help', component: RosblocksHelpComponent},
  {path: '**', component: WorkspaceComponent}
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
