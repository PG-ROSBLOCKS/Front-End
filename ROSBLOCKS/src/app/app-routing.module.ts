import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { WorkspaceComponent } from './workspace/workspace.component';
import { BlocklyCanvasComponent } from './blockly-canvas/blockly-canvas.component';

const routes: Routes = [
  {path: 'test', component: BlocklyCanvasComponent}, //Inicio
  {path: '**', component: WorkspaceComponent} //Inicio
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
