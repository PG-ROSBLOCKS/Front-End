import { Component, OnInit, ElementRef, ViewChild, AfterViewInit } from '@angular/core';
import * as Blockly from 'blockly';

@Component({
  selector: 'app-blockly',
  templateUrl: './blockly.component.html',
  styleUrls: ['./blockly.component.css']
})
export class BlocklyComponent implements AfterViewInit {
  workspace: Blockly.WorkspaceSvg | undefined;

  // Toolbox inicial
  toolbox = {
    kind: 'categoryToolbox',
    contents: [
      {
        kind: 'category',
        name: 'Logic',
        contents: [
          { kind: 'block', type: 'controls_if' },
          { kind: 'block', type: 'logic_compare' },
          { kind: 'block', type: 'logic_operation' },
          { kind: 'block', type: 'logic_negate' },
          { kind: 'block', type: 'logic_boolean' },
        ],
      },
      {
        kind: 'category',
        name: 'Loops',
        contents: [
          { kind: 'block', type: 'controls_repeat_ext' },
          { kind: 'block', type: 'controls_whileUntil' },
          { kind: 'block', type: 'controls_for' },
          { kind: 'block', type: 'controls_flow_statements' },
        ],
      },
      {
        kind: 'category',
        name: 'Math',
        contents: [
          { kind: 'block', type: 'math_number' },
          { kind: 'block', type: 'math_arithmetic' },
          { kind: 'block', type: 'math_single' },
          { kind: 'block', type: 'math_trig' },
          { kind: 'block', type: 'math_random_int' },
        ],
      },
    ],
  };

  ngAfterViewInit(): void {
    this.initializeBlockly();
  }

  initializeBlockly(): void {
    this.workspace = Blockly.inject('blocklyDiv', {
      toolbox: this.toolbox,
      grid: {
        spacing: 20,
        length: 3,
        colour: '#ccc',
        snap: true,
      },
      zoom: {
        controls: true,
        wheel: true,
      },
    });
  }

  onSearch(event: any): void {
    const query = event.target.value.toLowerCase();

    // Filtrar categorías y bloques por el texto ingresado
    const filteredToolbox = {
      kind: 'categoryToolbox',
      contents: this.toolbox.contents
        .map((category: any) => {
          // Filtrar bloques dentro de la categoría
          const filteredContents = category.contents.filter((block: any) =>
            block.type.toLowerCase().includes(query)
          );

          // Incluir la categoría solo si tiene bloques coincidentes
          if (filteredContents.length > 0) {
            return { ...category, contents: filteredContents };
          }
          return null;
        })
        .filter((category: any) => category !== null), // Eliminar categorías vacías
    };

    // Actualizar el toolbox con los resultados filtrados
    this.workspace?.updateToolbox(filteredToolbox);
  }
}