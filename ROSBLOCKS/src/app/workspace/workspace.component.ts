import { Component, AfterViewInit, OnInit, OnDestroy,ElementRef, ViewChild } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as Blockly from 'blockly';
import {pythonGenerator} from 'blockly/python';
import { definirBloquesROS2, definirGeneradoresROS2 } from '../blocks/ros2-blocks';
import { CodeService } from '../services/codeService';
import { Subscription } from 'rxjs';
import { switchMap } from 'rxjs/operators';
import { sanitizePythonFilename } from '../utilities/sanitizer-tools';
import { create_publisher } from '../blocks/code-generator';

@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements AfterViewInit, OnInit, OnDestroy {
  @ViewChild('resizer') resizer!: ElementRef;
  @ViewChild('leftSection') leftSection!: ElementRef;
  @ViewChild('rightSection') rightSection!: ElementRef;

  isResizing = false;

  handleMouseMove = (event: MouseEvent) => {
    if (!this.isResizing) return;
    let newWidth = event.clientX;
    let containerWidth = document.getElementById('workspace-container')!.offsetWidth;

    if (newWidth > 100 && newWidth < containerWidth * 0.8) {
      this.leftSection.nativeElement.style.width = `${newWidth}px`;
      this.rightSection.nativeElement.style.flex = '1'; // Mantiene la derecha flexible
    }
    if(this.selectedTabId) {
      this.selectTab(this.selectedTabId)
    }
  };

  stopResizing = () => {
    this.isResizing = false;
    document.removeEventListener('mousemove', this.handleMouseMove);
    document.removeEventListener('mouseup', this.stopResizing);
    
      
  };

  autoScrollEnabled: boolean = true;
  constructor(private http: HttpClient) {}
  // TEST 
  ngOnInit(): void {
    
  }
  ngOnDestroy(): void {
    for (const ws in this.websockets) {
      this.websockets.get(ws)?.unsubscribe();
    }    
  }
  // END TEST AREA
  private previousNames = new Map<number, string>(); // Almacena nombres anteriores por tabId
  MAX_NUM_PESTANAS = 8; // Max number of tabs
  consoles_output: Map<string, string> = new Map(); // Console outputs for each tab
  consoles_sessions: Map<string, string> = new Map(); // Sessions for each tab
  consoles_services: Map<string, CodeService> = new Map(); // Services for each tab
  websockets: Map<string, Subscription> = new Map(); // Websockets subscriptions for each tab
  text_code: Map<string, string> = new Map(); // Tab code
  current_displayed_console_output: string = ''; // Current console output
  codigo_testeo_backend: string = ''; // Test output for backend
  workspaces: { [key: number]: Blockly.WorkspaceSvg } = {}; // Diccionary for workspaces by tab id
  
  toolbox = {
    kind: 'categoryToolbox',
    contents: [
      {
        kind: 'category',
        name: 'Nodos',
        contents: [
          { kind: 'block', type: 'ros2_timer' },
          { kind: 'block', type: 'ros2_log' },
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
          { kind: 'block', type: 'ros2_create_publisher' },
          { kind: 'block', type: 'ros2_create_subscriber' },
          { kind: 'block', type: 'ros2_publish_message' },
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
    const resizer = this.resizer.nativeElement;
    const leftSection = this.leftSection.nativeElement;
    const rightSection = this.rightSection.nativeElement;

    resizer.addEventListener('mousedown', (event: MouseEvent) => {
      this.isResizing = true;
      document.addEventListener('mousemove', this.handleMouseMove);
      document.addEventListener('mouseup', this.stopResizing);
    });

    if (this.tabs.length > 0) {
      this.selectTab(this.tabs[0].id);
    }
    const consoleContainer = document.querySelector('.console-output-container');

    if (consoleContainer) {
      consoleContainer.addEventListener('scroll', () => {
        // Verifica si el usuario ha desplazado la consola manualmente
        const isAtBottom =
          consoleContainer.scrollHeight - consoleContainer.scrollTop <= consoleContainer.clientHeight + 5;

        if (isAtBottom) {
          this.autoScrollEnabled = true; // Reactiva el scroll automático si el usuario está en la parte inferior
        } else {
          this.autoScrollEnabled = false; // Desactiva el scroll automático si el usuario está desplazándose manualmente
        }
      });
    }
  }

  initializeBlockly(tabId: number): void {
    const blocklyDivId = `blocklyDiv-${tabId}`;
    const blocklyDiv = document.getElementById(blocklyDivId);
    // Definir bloques personalizados
    definirBloquesROS2();

    // Definir generadores de código
    definirGeneradoresROS2();

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

    const newTabId = Date.now(); // ID basado en timestamp
    

    this.tabs.push({ name: this.getUniqueTabName(), id: newTabId, isPlaying: false });
    this.consoles_services.set(newTabId.toString(), new CodeService(this.http));

    setTimeout(() => {
        this.selectTab(newTabId);
    }, 0);
}

getUniqueTabName(): string {
  let baseName = "Nodo";
  let newTabName = "";
  let index = 1;

    // Encuentra un nombre único en formato "Nodo_#"
    do {
        newTabName = sanitizePythonFilename(`${baseName}_${index}`).replace(/\.py$/, "");
        index++;
    } while (this.tabs.some(tab => tab.name === newTabName))
  return newTabName
}


  selectTab(tabId: number) {
    this.selectedTabId = tabId;
    setTimeout(() => {
      this.initializeBlockly(tabId);
    }, 0);
    this.codigo_testeo_backend = this.text_code.get(tabId.toString()) || ''; // TEST
    this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) || '';
  }

  storePreviousName(tab: any) {
    this.previousNames.set(tab.id, tab.name);
  }

  // Función para cambiar el nombre de la pestaña
  changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;

    const previousName = this.previousNames.get(tabId) || tab.name; // Recupera el nombre anterior

    // Sanitiza el nombre y remueve la extensión `.py`
    let sanitizedNewName = sanitizePythonFilename(newName).replace(/\.py$/, "");

    if (!sanitizedNewName) {
        alert('El nombre de la pestaña no puede estar vacío.');
        tab.name = previousName; // Restaura el nombre anterior
        return;
    }

    if (this.tabs.some(t => t.name === sanitizedNewName && t.id !== tabId)) {
        alert('Ya existe una pestaña con ese nombre.');
        tab.name = previousName; // Restaura el nombre anterior
        return;
    }

    tab.name = sanitizedNewName; // Asigna el nombre sanitizado
}


  playTab(tabId: number, playAllTabs: boolean) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    
    if (!tab) return; // Si el tab no existe, no hace nada
    // TODO: tests with new blocks
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.text_code.set(tabId.toString(), pythonGenerator.workspaceToCode(this.workspaces[tabId]));
  }

    tab.isPlaying = playAllTabs ? true : !tab.isPlaying; // Alterna solo si no es "play all"

    tab.isPlaying
        ? this.executeCode(this.text_code.get(tabId.toString()) || '', tabId)
        : this.stopTab(tabId);
}


  stopTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.isPlaying = false; 
      const session_id = this.consoles_sessions.get(tabId.toString());
      console.log('Session ID stop:', session_id);
      if(session_id) {
        this.consoles_services.get(tabId.toString())?.killExecution(session_id); 
      }
      this.consoles_services.get(tabId.toString())?.closeConnection();
      this.websockets.get(tabId.toString())?.unsubscribe();
    }
  }

  deleteTab(tabId: number) {
    if (this.workspaces[tabId]) {
      this.workspaces[tabId].dispose(); // Deletes workspace from blockly
      delete this.workspaces[tabId]; // Removes from object
      this.consoles_output.delete(tabId.toString()); // Deletes console
      this.consoles_sessions.delete(tabId.toString()); // Deletes session
      this.consoles_services.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || ''); // Deletes file
      this.consoles_services.delete(tabId.toString()); // Deletes service
      this.websockets.delete(tabId.toString()); // Deletes websocket
      this.text_code.delete(tabId.toString()); // Deletes code
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

  executeCode(code: string, tabId?: number) {    
    if (tabId !== null && tabId !== undefined) {
      this.enviarCodigo(code, tabId);
    }
  }

  playAllTabs() {
    for (const tab of this.tabs) {
      const tabId = tab.id;  
      if (!this.workspaces[tabId]) continue;
      this.playTab(tabId, true)  
    }
  }

  stopAllTabs() {
    for (const tab of this.tabs) {
      const tabId = tab.id;  
      if (!this.workspaces[tabId]) continue;
      this.stopTab(tabId)  
    }
  }

  cleanConsole() {
    if (this.current_displayed_console_output != '' && this.selectedTabId) {
      this.consoles_output.set(this.selectedTabId.toString(), '');
      this.current_displayed_console_output = 'Consola limpia';
    }
  }

  enviarCodigo(code_to_send: string, tabId: number) {
    console.log('Enviando código...');
    const fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Nodo');
    const codeService = this.consoles_services.get(tabId.toString());
  const code = create_publisher(code_to_send, fileName);
  if(codeService === undefined) {
    console.error('No se encontró el servicio para la pestaña', tabId);
    return;
  } else { 
    if(this.websockets.get(tabId.toString())) {
      this.websockets.get(tabId.toString())?.unsubscribe();
    }
    this.websockets.set(tabId.toString(), codeService.uploadCode(fileName, code)
      .pipe(
        switchMap(() => codeService.executeCode(fileName)),
        switchMap((response) => {
          console.log('Respuesta del backend:', response);
          const sessionId = response.session_id;
          this.consoles_sessions.set(tabId.toString(), sessionId);
          console.log('Session ID:', sessionId);
          return codeService.connectToWebSocket(sessionId);
        })
      )
      .subscribe({
        next: (response) => {
          console.log('Mensaje WebSocket:', response.output);
          if (response.output != this.consoles_output.get(tabId.toString())) {
            this.consoles_output.set(tabId.toString(), (this.consoles_output.get(tabId.toString()) ?? '') + response.output + '\n'); //Save output
            if(this.selectedTabId === tabId) {
              this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) ?? ''; //Update displayed output
            }
            if (this.autoScrollEnabled) {
              setTimeout(() => this.scrollToBottom(), 100);
            }
          }        
        },
        error: (error) => console.error('Error:', error),
        complete: () => console.log('Proceso completado')
      }))
  }
}
  scrollToBottom() {
    const consoleContainer = document.querySelector('.console-output-container');
    if (consoleContainer) {
        consoleContainer.scrollTop = consoleContainer.scrollHeight;
    }
  }  
}


