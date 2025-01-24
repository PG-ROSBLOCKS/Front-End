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
  
    ngAfterViewInit(): void {
      this.initializeBlockly();
    }
  
    initializeBlockly(): void {
      this.workspace = Blockly.inject('blocklyDiv', {
        toolbox: this.toolbox,
        trashcan: true,
        /*
              grid: {
          spacing: 20,
          length: 3,
          colour: '#ccc',
          snap: true
        },
        */
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
  
      // Actualizar el toolbox con los resultados filtrados
      this.workspace?.updateToolbox(filteredToolbox);
    }

  tabs: { name: string, id: number }[] = [];
  selectedTabId: number | null = null;
  selectedTabName: string | null = null;

  addTab() {
    const newTabId = this.tabs.length + 1;
    this.tabs.push({ name: `Nodo ${newTabId}`, id: newTabId });
    this.selectTab(newTabId);
  }

  selectTab(tabId: number) {
    this.selectedTabId = tabId;
    const selectedTab = this.tabs.find(tab => tab.id === tabId);
    this.selectedTabName = selectedTab ? selectedTab.name : null;
  }

  changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.name = newName;
    }
  }

  playTab(tabId: number) {

  }

  deleteTab(tabId: number) {
    this.tabs = this.tabs.filter(tab => tab.id !== tabId);
    if (this.selectedTabId === tabId) {
      this.selectedTabId = this.tabs.length > 0 ? this.tabs[0].id : null;
    }
  }
}