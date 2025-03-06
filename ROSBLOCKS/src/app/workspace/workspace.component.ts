import { AlertService } from './../shared/components/alert/alert.service';
import { Component, AfterViewInit, OnInit, OnDestroy, ElementRef, ViewChild } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import { definirBloquesROS2, definirGeneradoresROS2 } from '../blocks/ros2-blocks';
import { CodeService } from '../services/code.service';
import { Subscription } from 'rxjs';
import { switchMap } from 'rxjs/operators';
import { extractFirstLine, extractServiceFilename, replaceServiceFilename, sanitizePythonFilename, sanitizeSrvFilename, sanitizeMsgFilename, extractMessageFilename, replaceMessageFilename } from '../utilities/sanitizer-tools';
import { create_publisher, create_server } from '../blocks/code-generator';
import { of } from 'rxjs';
import { srvList, SrvInfo } from '../shared/srv-list';

@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements OnInit, OnDestroy {
  @ViewChild('resizer') resizer!: ElementRef;
  @ViewChild('leftSection') leftSection!: ElementRef;
  @ViewChild('rightSection') rightSection!: ElementRef;

  isResizing = false;
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
  autoScrollEnabled: boolean = true;
  tabs: { name: string; id: number; isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;


  constructor(
    private http: HttpClient,
    private codeService: CodeService,
    private alertService: AlertService
  ) { }

  // TEST 
  ngOnInit(): void {

  }
  ngOnDestroy(): void {
    for (const ws in this.websockets) {
      this.websockets.get(ws)?.unsubscribe();
    }
  }
  // END TEST AREA

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
          { kind: 'block', type: 'ros2_service_block' },
          { kind: 'block', type: 'ros2_named_message' },
          { kind: 'block', type: 'ros_create_server' },
        ],
      },
      {
        kind: 'category',
        name: 'Topicos',
        contents: [
          { kind: 'block', type: 'ros2_create_publisher' },
          { kind: 'block', type: 'ros2_minimal_publisher' },
          { kind: 'block', type: 'ros2_create_subscriber' },
          { kind: 'block', type: 'ros2_subscriber_msg_data' },
          { kind: 'block', type: 'ros2_turtlesim_pose_field' },
          { kind: 'block', type: 'ros2_print_msg_type' },
          { kind: 'block', type: 'ros2_publish_message' },
        ],
      },
      {
        kind: 'category',
        name: 'Mensajes',
        contents: [
          { kind: 'block', type: 'ros2_message_block' },
          { kind: 'block', type: 'ros2_named_message' }
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


    // Agrega el listener para detectar solo los eventos relevantes:
    this.workspaces[tabId].addChangeListener((event) => {
      if (event.type === Blockly.Events.BLOCK_CHANGE) {
        this.codeService.setWorkspaceChanged(true);
        console.log('Evento detectado: BLOCK_CHANGE. Flag actualizado.');
      } else if (event.type === Blockly.Events.BLOCK_CREATE) {
        this.codeService.setWorkspaceChanged(true);
        console.log('Evento detectado: BLOCK_CREATE. Flag actualizado.');
      } else if (event.type === Blockly.Events.BLOCK_DELETE) {
        this.codeService.setWorkspaceChanged(true);
        console.log('Evento detectado: BLOCK_DELETE. Flag actualizado.');
      }
      //TODO: Cuando se conecta un bloque a otro        
    });

    //ELIMINAR SUSCRIPTOR y PUBLICADOR
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE) {
        //Verificar si es instancia de BlockDelete
        if (event instanceof Blockly.Events.BlockDelete) {
          console.log('XML antiguo:', event.oldXml);
          //Convertir el XML a cadena 
          if (event.oldXml) {
            let xmlString = Blockly.Xml.domToText(event.oldXml);
            //Verificar si el type es 'ros2_create_subscriber'
            if (xmlString.includes('ros2_create_subscriber') || xmlString.includes('ros2_minimal_publisher') || xmlString.includes('ros2_create_publisher') || xmlString.includes('ros2_subscriber_msg_data') || xmlString.includes('ros2_publish_message')) {
              console.log('Bloque de publicador o suscriptor eliminado');
              //alerta acabas de eliminar un bloque de publicador o suscriptor, por ende la sesión terminará
              const resultado = await this.alertService.showAlert('Acabas de eliminar un bloque de publicador o suscriptor, por ende la sesión terminará');
              console.log('El usuario presionó OK:', resultado);
              this.stopTab(tabId);
              //eliminamos 
              this.consoles_sessions.delete(tabId.toString()); // Deletes session
              this.consoles_services.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || ''); // Deletes file
            }
          }
        }
      }
      //realizamos un conteo de bloques en el workspace
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // Listener para verificar la creación de bloques de tipo "ros_create_server"
    this.workspaces[tabId].addChangeListener(async (event) => {
      // Verificamos que el evento sea de tipo BLOCK_CREATE y sea una instancia de BLOCK_CREATE
      if (event.type === Blockly.Events.BLOCK_CREATE && event instanceof Blockly.Events.BlockCreate) {
        // En eventos de creación, la propiedad con la definición del bloque es "xml"
        if (event.xml) {
          const xmlString = Blockly.Xml.domToText(event.xml);
          // Verificamos si el bloque creado es de tipo "ros_create_server"
          if (xmlString.includes("ros_create_server")) {
            // Realizamos la verificación al backend a través del CodeService
            this.codeService.checkSrvFiles().subscribe(response => {
              if (!response.exists) {
                // Si no existen archivos .srv, mostramos la alerta al usuario
                this.alertService.showAlert("No se han encontrado archivos .srv en el proyecto. Deberías crear un servicio antes de usar el bloque 'Crear Servidor'.");
              }
            }, error => {
              console.error("Error al consultar archivos .srv:", error);
            });
          }
        }
      }
    });


    // Creates output console
    this.consoles_output.set(tabId.toString(), '');

    //Creates tab code
    this.text_code.set(tabId.toString(), '');
  }

  async addTab() {
    if (this.tabs.length >= this.MAX_NUM_PESTANAS) {
      const resultado = await this.alertService.showAlert('No se pueden agregar más de ' + this.MAX_NUM_PESTANAS + ' pestañas.');
      return;
    }
    const newTabId = Date.now(); // ID basado en timestamp
    this.tabs.push({ name: this.getUniqueTabName(), id: newTabId, isPlaying: false });
    this.consoles_services.set(newTabId.toString(), new CodeService(this.http));
    setTimeout(() => {
      this.selectTab(newTabId);
    }, 0);
    this.codeService.setNoTabs(false);
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
  async changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;
    const previousName = this.previousNames.get(tabId) || tab.name; // Recupera el nombre anterior
    // Sanitiza el nombre y remueve la extensión `.py`
    let sanitizedNewName = sanitizePythonFilename(newName).replace(/\.py$/, "");
    if (!sanitizedNewName) {
      const resultado = await this.alertService.showAlert('El nombre de la pestaña no puede estar vacío.');
      tab.name = previousName; // Restaura el nombre anterior
      return;
    }
    if (this.tabs.some(t => t.name === sanitizedNewName && t.id !== tabId)) {
      const resultado = await this.alertService.showAlert('Ya existe una pestaña con ese nombre.');
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
    // Actualiza la lista de archivos .srv antes de ejecutar la pestaña
    this.updateSrvList();
    tab.isPlaying = playAllTabs ? true : !tab.isPlaying; // Alterna solo si no es "play all"
    tab.isPlaying
      ? (this.executeCode(this.text_code.get(tabId.toString()) || '', tabId),
        this.codeService.setWorkspaceChanged(false))
      : this.stopTab(tabId);
  }


  stopTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.isPlaying = false;
      const session_id = this.consoles_sessions.get(tabId.toString());
      console.log('Session ID stop:', session_id);
      if (session_id) {
        this.consoles_services.get(tabId.toString())?.killExecution(session_id);
      }
      this.consoles_services.get(tabId.toString())?.closeConnection();
      this.websockets.get(tabId.toString())?.unsubscribe();
    }
  }

  deleteTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
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
    if (this.tabs.length === 0) {
      this.codeService.setNoTabs(true);
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
    // Verifica que en el workspace del tab actual existan bloques.
    const workspace = this.workspaces[tabId];
    if (!workspace) {
      console.error('No existe la workspace para el tab', tabId);
      return;
    }
    var code = '';
    var fileName = '';
    var type = '';
    // firstLine: refiere al tipo, remainingText: refiere al código
    const { firstLine, remainingText } = extractFirstLine(code_to_send);
    type = firstLine;
    code = remainingText;
    if (type == "pub_sub") {
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Nodo');
      code = create_publisher(code, fileName);
    } else if (type == "server") {  // Usamos "server" para identificar los nodos servidores
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Servidor');
      code = create_server(code, fileName);
    } else if (type == "srv") {
      fileName = sanitizeSrvFilename(extractServiceFilename(code) || 'Servicio.srv');
      code = replaceServiceFilename(code, fileName);
    } else if (type == "msg") {
      fileName = sanitizeMsgFilename(extractMessageFilename(code) || 'FailedMsg.msg');
      code = replaceMessageFilename(code, fileName);
    }
    const codeService = this.consoles_services.get(tabId.toString());

    if (codeService === undefined) {
      console.error('No se encontró el servicio para la pestaña', tabId);
      return;
    } else {
      if (this.websockets.get(tabId.toString())) {
        this.websockets.get(tabId.toString())?.unsubscribe();
      }
      this.websockets.set(tabId.toString(), codeService.uploadCode(fileName, code, type)
        .pipe(
          switchMap(() => {
            if (type === "srv") {
              console.log("El archivo es un servicio (.srv), deteniendo ejecución después de uploadCode.");
              const confirmationMessage = `Servicio ${fileName} creado correctamente.`;
              this.consoles_output.set(tabId.toString(),
                (this.consoles_output.get(tabId.toString()) ?? '') + confirmationMessage + '\n');
              // Actualizar la consola en la pestaña activa
              if (this.selectedTabId === tabId) {
                this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) ?? '';
              }
              // Asegurar auto-scroll si está habilitado
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
              return of(null); // Se detiene la ejecución del pipe aquí
            } else if (type === "msg") {
              console.log("El archivo es un servicio (.msg), deteniendo ejecución después de uploadCode.");
              const confirmationMessage = `Mensaje ${fileName} creado correctamente.`;
              this.consoles_output.set(tabId.toString(),
                (this.consoles_output.get(tabId.toString()) ?? '') + confirmationMessage + '\n');
              // Actualizar la consola en la pestaña activa
              if (this.selectedTabId === tabId) {
                this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) ?? '';
              }
              // Asegurar auto-scroll si está habilitado
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
              return of(null); // Se detiene la ejecución del pipe aquí
            }
            return codeService.executeCode(fileName); // Solo ejecuta si es pub_sub
          }),
          switchMap((response) => {
            if (!response) return of(null); // Si response es null (por ser .srv), no sigue ejecutando

            console.log('Respuesta del backend:', response);
            const sessionId = response.session_id;
            this.consoles_sessions.set(tabId.toString(), sessionId);
            console.log('Session ID:', sessionId);
            return codeService.connectToWebSocket(sessionId);
          })
        )
        .subscribe({
          next: (response) => {
            if (!response) return; // Si es null, no hace nada más
            console.log('Mensaje WebSocket:', response.output);
            if (response.output != this.consoles_output.get(tabId.toString())) {
              this.consoles_output.set(tabId.toString(), (this.consoles_output.get(tabId.toString()) ?? '') + response.output + '\n');
              if (this.selectedTabId === tabId) {
                this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) ?? '';
              }
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
            }
          },
          error: (error) => console.error('Error:', error),
          complete: () => console.log('Proceso completado')
        }));
    }
  }
  scrollToBottom() {
    const consoleContainer = document.querySelector('.console-output-container');
    if (consoleContainer) {
      consoleContainer.scrollTop = consoleContainer.scrollHeight;
    }
  }

  candidateTabToDelete: number | null = null;
  // Propiedad para almacenar el nombre del tab candidato a eliminar
  candidateTabName: string = '';

  confirmDeleteTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      this.candidateTabToDelete = tabId;
      this.candidateTabName = tab.name;
    }
  }

  handleDeleteConfirmation(response: boolean) {
    const tabId = this.candidateTabToDelete;
    this.candidateTabToDelete = null;
    if (response && tabId) {
      this.deleteTab(tabId)
    }
  }

  imageSrc: string | ArrayBuffer | null = null;

  onFileSelected(event: Event): void {
    const target = event.target as HTMLInputElement;
    if (target.files && target.files[0]) {
      const file = target.files[0];
      const reader = new FileReader();
      reader.onload = () => {
        // Al leer la imagen se asigna el resultado a la variable imageSrc
        this.imageSrc = reader.result;
      };
      reader.readAsDataURL(file);
    }
  }
  getConsoleLines(): string[] {
    return this.current_displayed_console_output
      ? this.current_displayed_console_output
        .trimEnd() // 🔹 Elimina espacios y saltos de línea extra al final
        .split('\n')
      : [];
  }
  

// Método auxiliar para crear un bloque 'srv_variable' con los campos de nombre y tipo.
private createSrvVariableBlock(variable: any) {
  return {
    kind: 'block',
    type: 'srv_variable',
    fields: {
      VAR_NAME: variable.name,    // Se mostrará en lugar de "nombreVar"
      VAR_TYPE: variable.type     // Se mostrará en lugar de "tipoVar"
    }
  };
}

// Función para actualizar la categoría "Variables de Servicio" en el toolbox
updateSrvVariablesCategory(): void {
  const toolboxObj = this.toolbox.contents && this.toolbox.contents.length > 0
    ? { ...this.toolbox }
    : { kind: 'categoryToolbox', contents: [] };

    const srvVariablesCategory = {
      kind: 'category',
      type: 'category',
      name: 'Variables de Servicio',
      contents: srvList.map((service: SrvInfo) => {
        // Extraer variables de request y response
        const requestBlocks = service.variables?.request?.map((variable: any) => this.createSrvVariableBlock(variable)) || [];
        const responseBlocks = service.variables?.response?.map((variable: any) => this.createSrvVariableBlock(variable)) || [];
    
        return {
          kind: 'category',
          type: 'category',
          // Se muestra solo el nombre del servicio sin extensión
          name: service.name ? service.name.replace(/\.srv$/, "") : "",
          contents: [
            { kind: 'label', text: "Solicitud:" },
            ...requestBlocks,
            { kind: 'label', text: "Respuesta:" },
            ...responseBlocks
          ]
        };
      })
    };

  const contents = toolboxObj.contents;
  const existingIdx = contents.findIndex((cat: any) => cat.name === "Variables de Servicio");
  if (existingIdx !== -1) {
    contents[existingIdx] = srvVariablesCategory;
  } else {
    contents.push(srvVariablesCategory);
  }

  if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
    this.workspaces[this.selectedTabId].updateToolbox({
      kind: 'categoryToolbox',
      contents: contents
    });
  }
}

  

  // Función para actualizar la lista de archivos srv
  updateSrvList(): void {
    this.codeService.checkSrvFiles().subscribe(response => {
      if (response.exists) {
        // Vaciar la lista global y poblarla con los objetos recibidos
        srvList.length = 0;
        response.files.forEach((file: any) => {
          // Aquí se asume que el backend ya devuelve el formato adecuado
          srvList.push(file);
        });
      } else {
        srvList.length = 0;
      }
      console.log("srvList actualizada:", srvList);
      // Si deseas también actualizar alguna categoría en el toolbox con las variables,
      // puedes invocar un método para reconstruir esa parte del toolbox.
      this.updateSrvVariablesCategory();
    }, error => {
      console.error("Error al obtener la lista de archivos srv:", error);
      srvList.length = 0;
    });
  }
  
}

