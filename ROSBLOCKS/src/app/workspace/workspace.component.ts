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
  MAX_NUM_PESTANAS = 8; // Máximo número de pestañas permitidas
  consoles_output: Map<string, string> = new Map(); // Salida de la consola por pestaña
  current_displayed_console_output: string = ''; // Salida de la consola ACTUAL
  codigo_testeo_backend: string = ''; // Código a enviar al backend para ejecutar
  workspaces: { [key: number]: Blockly.WorkspaceSvg } = {}; // Diccionario de workspaces por ID de pestaña
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

    // Si ya existe un workspace para esta pestaña, solo lo redimensionamos
    if (this.workspaces[tabId]) {
      Blockly.svgResize(this.workspaces[tabId]);
      return;
    }

    // Si no existe, lo creamos
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
      theme: Blockly.Themes.Classic
    });
    // Se crea su consola de salida
    this.consoles_output.set(tabId.toString(), '');
  }

  addTab() {
    if (this.tabs.length >= this.MAX_NUM_PESTANAS) {
      alert('No se pueden agregar más de' + ' pestañas.');
      return;
    }

    const newTabId = Date.now(); // ID único basado en timestamp
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
      tab.isPlaying = !tab.isPlaying; // Alterna entre play y stop
    }
    // TODO: tests con los nuevos bloques
    // Generación de código python 
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.codigo_testeo_backend = pythonGenerator.workspaceToCode(this.workspaces[this.selectedTabId]);
    }
  }

  deleteTab(tabId: number) {
    // Borrar el workspace asociado
    if (this.workspaces[tabId]) {
      this.workspaces[tabId].dispose(); // Elimina el workspace de Blockly
      delete this.workspaces[tabId]; // Remueve del objeto
      this.consoles_output.delete(tabId.toString()); // Elimina la consola de salida
    }

    // Eliminar la pestaña
    this.tabs = this.tabs.filter(tab => tab.id !== tabId);

    // Si la pestaña eliminada estaba seleccionada, cambiar a otra
    if (this.selectedTabId === tabId) {
      this.selectedTabId = this.tabs.length > 0 ? this.tabs[0].id : null;
      if (this.selectedTabId) {
        this.selectTab(this.selectedTabId);
      }
    }
  }

  onSearch(event: any): void {
    const query = event.target.value.toLowerCase();

    // Filtrar categorías y bloques por el texto ingresado
    const filteredToolbox = {
      kind: 'categoryToolbox',
      contents: this.toolbox.contents
        .map((category: any) => {
          const filteredContents = category.contents.filter((block: any) =>
            block.type.toLowerCase().includes(query)
          );

          return filteredContents.length > 0 ? { ...category, contents: filteredContents } : null;
        })
        .filter((category: any) => category !== null), // Eliminar categorías vacías
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
}
