import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { WorkspaceComponent } from './workspace/workspace.component';
import { RosblocksHelpComponent } from './rosblocks-help/rosblocks-help.component';
import { LandingComponent } from './landing/landing.component';
import { LandingVisitedGuard } from './guards/landing-visited.guard';

const routes: Routes = [
  { path: '', component: LandingComponent, pathMatch: 'full' },
  { path: 'help', component: RosblocksHelpComponent },
  { path: 'workspace', 
    component: WorkspaceComponent,
    canActivate: [LandingVisitedGuard],
    data: { reuse: true }  
  }
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
