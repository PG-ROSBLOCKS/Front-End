import { Component, AfterViewInit, ViewChild, ElementRef, Renderer2 } from '@angular/core';
import * as Blockly from 'blockly';

@Component({
  selector: 'app-blockly-canvas',
  templateUrl: './blockly-canvas.component.html',
  styleUrls: ['./blockly-canvas.component.css']
})
export class BlocklyCanvasComponent implements AfterViewInit {

  @ViewChild('blocklyDiv', { static: true }) blocklyDiv!: ElementRef;

  constructor(private renderer: Renderer2) { }

  ngAfterViewInit(): void {
    this.initializeBlockly();
  }

  initializeBlockly(): void {
    // Aplicar estilos directamente en el componente
    this.applyToolboxStyles();

    const workspace = Blockly.inject(this.blocklyDiv.nativeElement, {
      toolbox: `  
        <xml xmlns="https://developers.google.com/blockly/xml">
          <!-- Sección de ejemplo de bloques -->
          <category name="Nodes" colour="120">
            <block type="controls_if"></block>
            <block type="controls_repeat_ext"></block>
            <block type="controls_whileUntil"></block>
            <block type="controls_forEach"></block>
            <block type="controls_flow_statements"></block>
          </category>
          <category name="Services" colour="230">
            <block type="math_number"></block>
            <block type="math_arithmetic"></block>
            <block type="math_round"></block>
            <block type="math_trig"></block>
            <block type="math_constant"></block>
            <block type="math_random_int"></block>
          </category>
          <category name="Topics" colour="210">
            <block type="logic_compare"></block>
            <block type="logic_operation"></block>
            <block type="logic_negate"></block>
            <block type="logic_null"></block>
            <block type="logic_boolean"></block>
          </category>
          <category name="Etc" colour="330">
            <block type="variables_get"></block>
            <block type="variables_set"></block>
            <block type="variables_create"></block>
          </category>
        </xml>`,
      media: 'https://unpkg.com/blockly/media/',
      trashcan: true,
      horizontalLayout: false,
    });

    // Asegurarse de que la Toolbox y el área de trabajo estén configuradas correctamente después de la inicialización
    setTimeout(() => {
      this.applyToolboxStyles();
      this.applyWorkspaceStyles(); // Aplicamos el estilo del workspace
    }, 500); // Esperar medio segundo para asegurarnos de que Blockly haya cargado
  }

  // Función que aplica los estilos CSS directamente con Renderer2
  applyToolboxStyles() {

    const toolboxDiv = this.blocklyDiv.nativeElement.querySelector('.blocklyToolboxDiv');
    if (toolboxDiv) {
      //this.renderer.setStyle(toolboxDiv, 'width', '50px');
      //this.renderer.setStyle(toolboxDiv, 'height', '100vh');
      this.renderer.setStyle(toolboxDiv, 'background-color', '#253942');
      this.renderer.setStyle(toolboxDiv, 'color', '#FFF');
      this.renderer.setStyle(toolboxDiv, 'display', 'flex');
      this.renderer.setStyle(toolboxDiv, 'flex-direction', 'column');
      this.renderer.setStyle(toolboxDiv, 'align-items', 'center');
      this.renderer.setStyle(toolboxDiv, 'justify-content', 'flex-start');
    }

    const categoryElements = this.blocklyDiv.nativeElement.querySelectorAll('.blocklyToolboxCategory');
    categoryElements.forEach((category: HTMLElement) => {
      //this.renderer.setStyle(category, 'width', '200px');
      //this.renderer.setStyle(category, 'padding', '0 50px');
      this.renderer.setStyle(category, 'margin', '5px 0px');
      this.renderer.setStyle(category, 'border-radius', '5px');
      this.renderer.setStyle(category, 'color', '#FFF');
    });

    const tree = this.blocklyDiv.nativeElement.querySelectorAll('.blocklyTreeRow');
    tree.forEach((category: HTMLElement) => {
      this.renderer.setStyle(category, 'background-color', '#39515c');
      this.renderer.setStyle(category, 'padding', '10px');
    });
  }

  applyWorkspaceStyles() {
    const workspaceDiv = this.blocklyDiv.nativeElement.querySelector('.blocklySvg');
    if (workspaceDiv) {
      this.renderer.setStyle(workspaceDiv, 'background-color', '#022741');
      this.renderer.setStyle(workspaceDiv, 'flex-grow', '1');
      this.renderer.setStyle(workspaceDiv, 'height', '100%');
      this.renderer.setStyle(workspaceDiv, 'background-image', 'url("https://www.freeiconspng.com/thumbs/grid-png/white-grid-png-13.png")');
    }
  }
}
