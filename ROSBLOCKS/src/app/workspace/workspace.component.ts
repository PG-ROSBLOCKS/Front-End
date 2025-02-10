import { Component, AfterViewInit } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as Blockly from 'blockly';
import {pythonGenerator} from 'blockly/python';

@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements AfterViewInit {
  constructor(private http: HttpClient) {}
  MAX_NUM_PESTANAS = 8; // Max number of tabs
  consoles_output: Map<string, string> = new Map(); // Console outputs for each tab
  current_displayed_console_output: string = ''; // Current console OUTPUT
  text_code: Map<string, string> = new Map(); // Tab code
  codigo_testeo_backend: string = ''; // Test output for backend
  workspaces: { [key: number]: Blockly.WorkspaceSvg } = {}; // Diccionary for workspaces by tab id
  
  toolbox = {
    kind: 'categoryToolbox',
    contents: [
      {
        kind: 'category',
        name: 'Nodos',
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
        name: 'Servicios',
        contents: [
          { kind: 'block', type: 'controls_repeat_ext' },
          { kind: 'block', type: 'controls_whileUntil' },
          { kind: 'block', type: 'controls_for' },
          { kind: 'block', type: 'controls_flow_statements' },
        ],
      },
      {
        kind: 'category',
        name: 'Topicos',
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
        name: 'Mensajes',
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
        name: 'Condicionales',
        contents: [
          { kind: 'block', type: 'controls_if' },
          { kind: 'block', type: 'controls_ifelse' },
          { kind: 'block', type: 'logic_compare' },
          { kind: 'block', type: 'logic_operation' },
          { kind: 'block', type: 'logic_negate' },
          { kind: 'block', type: 'logic_boolean' },
          { kind: "block", type: "logic_ternary" },
        ],
      },
      {
        kind: 'category',
        name: 'Ciclos',
        contents: [
          { kind: "block", type: "controls_repeat" }, // Repetir un número fijo de veces
          { kind: "block", type: "controls_repeat_ext" }, // Repetir con un número variable de veces
          { kind: "block", type: "controls_whileUntil" }, // Ciclo "while" o "until"
          { kind: "block", type: "controls_for" }, // Ciclo "for" con contador
          { kind: "block", type: "controls_forEach" }, // Ciclo "for each" para listas
        ],
      },
      {
        kind: 'category',
        name: 'Operaciones',
        contents: [
          { kind: "block", type: "math_number" }, // Número
          { kind: "block", type: "math_arithmetic" }, // Operaciones aritméticas (+, -, *, /)
          { kind: "block", type: "math_single" }, // Funciones matemáticas (raíz cuadrada, valor absoluto, etc.)
          { kind: "block", type: "math_trig" }, // Funciones trigonométricas (seno, coseno, tangente)
          { kind: "block", type: "math_round" }, // Redondeo (arriba, abajo, etc.)
          { kind: "block", type: "math_random_int" }, // Número aleatorio en un rango
          { kind: "block", type: "math_modulo" }, // Módulo (resto de la división)
        ],
      },
      {
        kind: 'category',
        name: 'Variables',
        contents: [
          { kind: "block", type: "variables_get" }, // Obtener el valor de una variable
          { kind: "block", type: "variables_set" }, // Asignar un valor a una variable
          /*{
            kind: "block",
            type: "variables_set_dynamic"
          }, // Asignar variable con nombre dinámico
          {
            kind: "block",
            type: "variables_get_dynamic"
          }*/ 
        ],
      },
      {
        kind: 'category',
        name: 'Funciones',
        contents: [
          { kind: "block", type: "procedures_defnoreturn" }, // Definir una función sin retorno
          { kind: "block", type: "procedures_defreturn" }, // Definir una función con retorno
          { kind: "block", type: "procedures_callnoreturn" }, // Llamar a una función sin retorno
          { kind: "block", type: "procedures_callreturn" }, // Llamar a una función con retorno
          { kind: "block", type: "procedures_ifreturn" }, // Retorno condicional en una función
        ],
      },
      {
        kind: 'category',
        name: 'Texto',
        contents: [
          { kind: "block", type: "text" }, // Texto
          { kind: "block", type: "text_join" }, // Concatenar texto
          { kind: "block", type: "text_append" }, // Agregar texto
          { kind: "block", type: "text_length" }, // Longitud de texto
          { kind: "block", type: "text_isEmpty" }, // Texto vacío
          { kind: "block", type: "text_indexOf" }, // Índice de texto
          { kind: "block", type: "text_charAt" }, // Carácter en posición
          { kind: "block", type: "text_getSubstring" }, // Subcadena
          { kind: "block", type: "text_changeCase" }, // Cambiar mayúsculas/minúsculas
          { kind: "block", type: "text_trim" }, // Quitar espacios en blanco
          { kind: "block", type: "text_print" }, // Imprimir texto
        ]
      }
    ],
  };

  tabs: { name: string; id: number; isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;

  ngAfterViewInit(): void {
    if (this.tabs.length > 0) {
      this.selectTab(this.tabs[0].id);
    }
  }

  initializeBlockly(tabId: number): void {
    const blocklyDivId = `blocklyDiv-${tabId}`;
    const blocklyDiv = document.getElementById(blocklyDivId);

    if (!blocklyDiv) return;

    // If a tab exist, just redimention
    if (this.workspaces[tabId]) {
      Blockly.svgResize(this.workspaces[tabId]);
      return;
    }

    const customTheme = Blockly.Theme.defineTheme('customTheme', {
      name: 'customTheme',
      base: Blockly.Themes.Classic,
      blockStyles: {
        logic_blocks: { colourPrimary: '#A55A83' }, // Nodos
        loop_blocks: { colourPrimary: '#3A8439' }, // Servicios
        math_blocks: { colourPrimary: '#3D65A8' }, // Topicos
        text_blocks: { colourPrimary: '#6835BB' }, // Mensajes
        conditional_blocks: { colourPrimary: '#569BBD' }, // Condicionales
        cycle_blocks: { colourPrimary: '#897099' }, // Ciclos
        operations_blocks: { colourPrimary: '#B28E34' }, // Operaciones
        variable_blocks: { colourPrimary: '#B46564' }, // Variables
        procedure_blocks: { colourPrimary: '#3E7E7E' }, // Funciones
        text_manipulation_blocks: { colourPrimary: '#E91E63' } // Texto
      }
    });
    
    
    this.workspaces[tabId] = Blockly.inject(blocklyDiv, {
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
      theme: customTheme // Apply the updated theme
    });
    
    // Creates output console
    this.consoles_output.set(tabId.toString(), '');

    //Creates tab code
    this.text_code.set(tabId.toString(), '');
  }

  addTab() {
    if (this.tabs.length >= this.MAX_NUM_PESTANAS) {
      alert('No se pueden agregar más de ' + this.MAX_NUM_PESTANAS + ' pestañas.');
      return;
    }
  
    const newTabId = Date.now(); // ID based un timestamp
    this.tabs.push({ name: `Nodo ${this.tabs.length + 1}`, id: newTabId, isPlaying: false });
  
    setTimeout(() => {
      this.selectTab(newTabId);
    }, 0);
  } 

  selectTab(tabId: number) {
    this.selectedTabId = tabId;
    setTimeout(() => {
      this.initializeBlockly(tabId);
    }, 0);
    this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) || ''; // TEST
    this.codigo_testeo_backend = this.text_code.get(tabId.toString()) || ''; // TEST
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
      tab.isPlaying = !tab.isPlaying; // Alternates play & stop
    }
    // TODO: tests with new blocks
    // Generación of Python code
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.text_code.set(tabId.toString(), pythonGenerator.workspaceToCode(this.workspaces[tabId]));
    }

    // Code execution
    if (tab !== undefined) {
      if (tab.isPlaying) {
        this.executeCode(this.text_code.get(tabId.toString()) || '');
      }
    }
  }

  deleteTab(tabId: number) {
    if (this.workspaces[tabId]) {
      this.workspaces[tabId].dispose(); // Deletes workspace from blockly
      delete this.workspaces[tabId]; // Removes from object
      this.consoles_output.delete(tabId.toString()); // Deletes console
    }

    //Deletes tab
    this.tabs = this.tabs.filter(tab => tab.id !== tabId);

    // Change to another tab
    if (this.selectedTabId === tabId) {
      this.selectedTabId = this.tabs.length > 0 ? this.tabs[0].id : null;
      if (this.selectedTabId) {
        this.selectTab(this.selectedTabId);
      }
    }
  }

  onSearch(event: any): void {
    const query = event.target.value.toLowerCase();

    // Filter categories by text
    const filteredToolbox = {
      kind: 'categoryToolbox',
      contents: this.toolbox.contents
        .map((category: any) => {
          const filteredContents = category.contents.filter((block: any) =>
            block.type.toLowerCase().includes(query)
          );

          return filteredContents.length > 0 ? { ...category, contents: filteredContents } : null;
        })
        .filter((category: any) => category !== null), // Deletes empty categories
    };

    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.workspaces[this.selectedTabId].updateToolbox(filteredToolbox);
    }
  }

  executeCode(code: string) {
    this.http.post<{ output: string; error: string }>('http://localhost:8000/execute', { code })
      .subscribe(response => {
        this.current_displayed_console_output += (response.output || response.error);
        if (this.selectedTabId !== null) {
          this.consoles_output.set(this.selectedTabId.toString(), this.current_displayed_console_output);
        }        
      });
  }

  playAllTabs() {
    for (const tab of this.tabs) {
      const tabId = tab.id;
  
      if (!this.workspaces[tabId]) continue;
  
      const code = pythonGenerator.workspaceToCode(this.workspaces[tabId]);
      this.text_code.set(tabId.toString(), code);
  
      this.http.post<{ output: string; error?: string }>('http://localhost:8000/execute', { code }).subscribe(
        (response) => {
          const output = response.output || response.error || "";
          const previousOutput = this.consoles_output.get(tabId.toString()) || "";
          this.consoles_output.set(tabId.toString(), previousOutput + output);
          // If current tab selected, actualize `current_displayed_console_output`
          if (this.selectedTabId === tabId) {
            this.current_displayed_console_output = this.consoles_output.get(tabId.toString())!;
          }
        },
        (error) => {
          console.error(`Error ejecutando código en pestaña ${tabId}:`, error);
        }
      );
    }
  }

  
}
