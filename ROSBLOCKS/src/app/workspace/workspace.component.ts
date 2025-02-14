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

  //Blocks we create must be specified here
  //TODO: Specify code in Python for the block
  ngAfterViewInit(): void {
    Blockly.defineBlocksWithJsonArray([
      {
        "type": "custom_action",
        "message0": "Bloque de prueba: %1 y %2",
        "args0": [
          {
            "type": "input_value",
            "name": "MESSAGE1",
            "check": "String"
          },
          {
            "type": "input_value",
            "name": "MESSAGE2",
            "check": "String"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 905,
        "tooltip": "Bloque de prueba",
        "helpUrl": ""
      },
      {
        "type": "timer",
        "message0": "Espera %1 milisegundos",
        "args0": [
          {
            "type": "input_value",
            "name": "TIME",
            "check": "Number"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 120,
        "tooltip": "Waits for a specified amount of time in milliseconds.",
        "helpUrl": "",
        "inputsInline": true
      },
      {
        "type": "c_if_else",
        "message0": "Custom service %1 Entrada %2 Salida %3",
        "args0": [
          {
            "type": "input_value",
            "name": "CONDITION",
            "check": "Boolean"
          },
          {
            "type": "input_statement",
            "name": "DO_IF"
          },
          {
            "type": "input_statement",
            "name": "DO_ELSE"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 230,
        "tooltip": "If else block in the shape of C.",
        "helpUrl": "",
        "inputsInline": false,
        "extensions": ["colours_custom"]
      }
    ]);

    Blockly.Blocks['c_if_else'].init = function() {
      this.appendValueInput("CONDITION")
          .setCheck("Boolean")
          .appendField("Custom Service");
    
      this.appendStatementInput("DO_IF")
          .setCheck(null)
          .appendField("Entrada");
    
      this.appendStatementInput("DO_ELSE")
          .setCheck(null)
          .appendField("Salida");
    
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
    
      var path = this.getSvgRoot().querySelector('path');
      if (path) {
        // Redefine la forma del bloque para que sea una "C"
        path.setAttribute('d', 'M 10,10 C 40,10 60,40 40,70 C 60,100 40,130 10,130 Z');
        path.setAttribute('fill', '#66ccff');
      }
      this.setStyle('colour_custom');
    };
    
    
    // Initialize Blockly after the view is set up
    if (this.tabs.length > 0) {
      this.selectTab(this.tabs[0].id);
    }
  }
  
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
      },
      {
        kind: 'category',
        name: 'Test',
        contents: [
          {kind: 'block',type: 'custom_action'},
          {kind: 'block',type: 'timer'},
          {kind: 'block',type: 'c_if_else'},
        ],
      }
    ],
  };

  tabs: { name: string; id: number; isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;

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
    if (this.tabs.length >= 8) {
        alert('No se pueden agregar más de 8 pestañas.');
        return;
    }

    const newTabId = Date.now();
    this.tabs.push({ name: this.getUniqueTabName(), id: newTabId, isPlaying: false });

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
    if (!tab) return;

    newName = newName.trim();

    const validNameRegex = /^[a-zA-Z0-9 ]+$/;

    if (!newName) {
        alert("El nombre no puede estar vacío.");
        tab.name = this.getUniqueTabName();
        return;
    }
    else if (!validNameRegex.test(newName)) {
        alert("El nombre solo puede contener letras (mayúsculas o minúsculas), números y espacios.");
        tab.name = this.getUniqueTabName();
        return;
    }
    else if (this.tabs.some(t => t.name === newName && t.id !== tab.id)) {
        alert("El nombre ya está en uso. Elige otro.");
        tab.name = this.getUniqueTabName();
        return;
    }
    else {
      tab.name = newName;
    }
    
}


  getUniqueTabName(): string {
    let index = 1;
    const existingNames = new Set(this.tabs.map(tab => tab.name));

    while (existingNames.has(`Nodo ${index}`)) {
        index++;
    }

    return `Nodo ${index}`;
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

  cleanConsole() {
    if (this.current_displayed_console_output != '') {
      this.current_displayed_console_output = 'Consola limpia';
    }
  }
}
