import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { WorkspaceComponent } from './workspace/workspace.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { LandingComponent } from './landing/landing.component';

const routes: Routes = [
  { path: '', component: LandingComponent, pathMatch: 'full' }, // ⬅︎ landing
  { path: 'help', component: RosblocksHelpComponent },
  { path: 'workspace', 
    component: WorkspaceComponent,
    data: { reuse: true }          // ⬅︎  marca que esta ruta debe reciclarse
  }
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
