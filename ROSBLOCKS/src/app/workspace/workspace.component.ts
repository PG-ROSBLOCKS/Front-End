import { Component, AfterViewInit} from '@angular/core';
import * as Blockly from 'blockly';

@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements AfterViewInit {
  workspace: Blockly.WorkspaceSvg | undefined;
  
  // Toolbox inicial (Los bloques aqui son solo de prueba)
  toolbox = {
    kind: 'categoryToolbox',
    contents: [
      {
        kind: 'category',
        name: 'Nodes',
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
        name: 'Services',
        contents: [
          { kind: 'block', type: 'controls_repeat_ext' },
          { kind: 'block', type: 'controls_whileUntil' },
          { kind: 'block', type: 'controls_for' },
          { kind: 'block', type: 'controls_flow_statements' },
        ],
      },
      {
        kind: 'category',
        name: 'Topics',
        contents: [
          { kind: 'block', type: 'math_number' },
          { kind: 'block', type: 'math_arithmetic' },
          { kind: 'block', type: 'math_single' },
          { kind: 'block', type: 'math_trig' },
          { kind: 'block', type: 'math_random_int' },
        ],
      },
      {
        kind: 'category',
        name: 'etc',
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

  tabs: { name: string, id: number, isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;
  selectedTabName: string | null = null;

  ngAfterViewInit(): void {
    // Inicialización de la primera pestaña
    if (this.tabs.length > 0) {
      this.selectTab(this.tabs[0].id);
    }
  }

  initializeBlockly(tabId: number): void {
    // El nombre del div será único por pestaña
    const blocklyDivId = `blocklyDiv-${tabId}`;
    this.workspace = Blockly.inject(blocklyDivId, {
      toolbox: this.toolbox,
      trashcan: true,
      zoom: {
        controls: true,
        wheel: true,
        startScale: 0.7,
        maxScale: 2,
        minScale: 0.6,
        scaleSpeed: 1.1
      },
      move: {
        scrollbars: true,
        drag: true,
        wheel: false
      },
      sounds: true,
      media: 'https://unpkg.com/blockly/media/',
      rtl: false,
      horizontalLayout: false,
      renderer: 'zelos',
      theme: Blockly.Themes.Classic
    });
  }

  addTab() {
    if (this.tabs.length >= 5) {
      alert('No se pueden agregar más de 5 pestañas.');
      return;
    }
    const newTabId = this.tabs.length + 1;
    this.tabs.push({ name: `Nodo ${newTabId}`, id: newTabId, isPlaying: false });

    // Selecciona la nueva pestaña y abre su espacio de trabajo
    this.selectTab(newTabId);
  }

  selectTab(tabId: number) {
    this.selectedTabId = tabId;
    this.selectedTabName = this.tabs.find(tab => tab.id === tabId)?.name || null;

    // Inicializar Blockly solo cuando se selecciona la pestaña
    this.initializeBlockly(tabId);
  }

  changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.name = newName;
    }
  }

  playTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.isPlaying = !tab.isPlaying; // Alterna entre true y false
    }
  }

  deleteTab(tabId: number) {
    this.tabs = this.tabs.filter(tab => tab.id !== tabId);
    if (this.selectedTabId === tabId) {
      this.selectedTabId = this.tabs.length > 0 ? this.tabs[0].id : null;
    }
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

          // Incluir la categoría solo si tiene bloques que coincidan
          if (filteredContents.length > 0) {
            return { ...category, contents: filteredContents };
          }
          return null;
        })
        .filter((category: any) => category !== null), // Eliminar categorías vacías
    };
  }
}