import { AlertService } from './../shared/components/alert/alert.service';
import { Component, AfterViewInit, OnInit, OnDestroy, ElementRef, ViewChild } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import { definirBloquesROS2, definirGeneradoresROS2 } from '../blocks/ros2-blocks';
import { CodeService } from '../services/code.service';
import { Subscription, of } from 'rxjs';
import { switchMap } from 'rxjs/operators';
import { extractFirstLine, extractServiceFilename, replaceServiceFilename, sanitizePythonFilename, sanitizeSrvFilename, sanitizeMsgFilename, extractMessageFilename, replaceMessageFilename } from '../utilities/sanitizer-tools';
import { create_client, create_publisher, create_server } from '../blocks/code-generator';
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
  private previousNames = new Map<number, string>();
  MAX_NUM_PESTANAS = 8;
  consoles_output: Map<string, string> = new Map();
  consoles_sessions: Map<string, string> = new Map();
  consoles_services: Map<string, CodeService> = new Map();
  websockets: Map<string, Subscription> = new Map();
  text_code: Map<string, string> = new Map();
  current_displayed_console_output: string = '';
  codigo_testeo_backend: string = '';
  workspaces: { [key: number]: Blockly.WorkspaceSvg } = {};
  autoScrollEnabled: boolean = true;
  tabs: { name: string; id: number; isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;

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
        name: 'Tópicos',
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
        name: 'Servicios',
        contents: [
          { kind: 'block', type: 'ros2_service_block' },
          { kind: 'block', type: 'ros2_named_message' },
          { kind: 'block', type: 'ros_create_server' },
          { kind: 'block', type: 'ros2_service_available' },
        ],
      },
      {
        kind: 'category',
        name: 'Clientes',
        contents: [
          { kind: 'block', type: 'ros_create_client' },
          { kind: 'block', type: 'ros_send_request' },
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
          { kind: "block", type: "controls_repeat" },
          { kind: "block", type: "controls_repeat_ext" },
          { kind: "block", type: "controls_whileUntil" },
          { kind: "block", type: "controls_for" },
          { kind: "block", type: "controls_forEach" },
        ],
      },
      {
        kind: 'category',
        name: 'Operaciones',
        contents: [
          { kind: "block", type: "math_number" },
          { kind: "block", type: "math_arithmetic" },
          { kind: "block", type: "math_single" },
          { kind: "block", type: "math_trig" },
          { kind: "block", type: "math_round" },
          { kind: "block", type: "math_random_int" },
          { kind: "block", type: "math_modulo" },
        ],
      },
      {
        kind: 'category',
        name: 'Variables',
        contents: [
          { kind: "block", type: "variables_get" },
          { kind: "block", type: "variables_set" },
        ],
      },
      {
        kind: 'category',
        name: 'Funciones',
        contents: [
          { kind: "block", type: "procedures_defnoreturn" },
          { kind: "block", type: "procedures_defreturn" },
          { kind: "block", type: "procedures_callnoreturn" },
          { kind: "block", type: "procedures_callreturn" },
          { kind: "block", type: "procedures_ifreturn" },
        ],
      },
      {
        kind: 'category',
        name: 'Texto',
        contents: [
          { kind: "block", type: "text" },
          { kind: "block", type: "text_join" },
          { kind: "block", type: "text_append" },
          { kind: "block", type: "text_length" },
          { kind: "block", type: "text_isEmpty" },
          { kind: "block", type: "text_indexOf" },
          { kind: "block", type: "text_charAt" },
          { kind: "block", type: "text_getSubstring" },
          { kind: "block", type: "text_changeCase" },
          { kind: "block", type: "text_trim" },
          { kind: "block", type: "text_print" },
        ]
      }
    ],
  };

  constructor(
    private http: HttpClient,
    private codeService: CodeService,
    private alertService: AlertService
  ) { }

  ngOnInit(): void {
    // Puedes registrar listeners globales o inicializar configuraciones aquí si lo requieres
  }

  ngOnDestroy(): void {
    for (const ws in this.websockets) {
      this.websockets.get(ws)?.unsubscribe();
    }
  }

  /**
   * Función global que recorre todas las workspaces (todas las tabs) y elimina
   * aquellos bloques de cliente cuyo campo SERVER_REF coincida con el SERVER_NAME pasado.
   */
  /*globalServerBlockDeleted(serverName: string): void {
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      const allBlocks = workspace.getAllBlocks();
      console.log(`Revisando tab ${tabKey} para clientes asociados a ${serverName}:`, allBlocks);
      allBlocks.forEach((block: any) => {
        if (block.type && block.type === 'ros_create_client') {
          const clientServerRef = block.getFieldValue('SERVICE_NAME');
          if (clientServerRef === serverName) {
            console.log(`Eliminando bloque de cliente ${block.id} en tab ${tabKey} asociado al servicio ${serverName}`);
            const blockToRemove = workspace.getBlockById(block.id);
            if (blockToRemove) {
              blockToRemove.dispose(true);
            }
            
            // Aquí podrías agregar la solicitud al backend para eliminar el nodo cliente en setup.py
          }
        }
      });
    });
  }

  
   * Registra el listener de eliminación de bloques de servidor para una workspace específica.
   * Cuando se elimina un bloque de servidor, se extrae el SERVER_NAME y se invoca la función
   * global para eliminar bloques de cliente asociados en TODAS las tabs.
   
  registerServerDeleteListenerForWorkspace(tabId: number): void {
    const workspace = this.workspaces[tabId];
    workspace.addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          // Verificar si se eliminó un bloque de servidor
          if (xmlString.includes('ros_create_server')) {
            console.log(`Bloque de servidor eliminado en tab ${tabId}`);
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(xmlString, "text/xml");
            const serverNameField = xmlDoc.querySelector('field[name="SERVER_NAME"]');
            const serverName = serverNameField ? serverNameField.textContent : null;
            console.log('XML:', xmlString);
            if (!serverName) {
              console.error('No se pudo obtener el SERVER_NAME del bloque de servidor eliminado.');
              return;
            }
            console.log(`SERVER_NAME extraído: ${serverName}`);
            // Solicitud al backend para eliminar el nodo servidor en setup.py

            // Recorrer TODAS las tabs para eliminar los bloques de cliente asociados
            this.globalServerBlockDeleted(serverName);
          }
        }
      }
      this.codeService.setNoBlocks(workspace.getAllBlocks().length === 0);
    });
  }*/

  initializeBlockly(tabId: number): void {
    const blocklyDivId = `blocklyDiv-${tabId}`;
    const blocklyDiv = document.getElementById(blocklyDivId);
    definirBloquesROS2();
    definirGeneradoresROS2();
    if (!blocklyDiv) return;
    if (this.workspaces[tabId]) {
      Blockly.svgResize(this.workspaces[tabId]);
      return;
    }
    const customTheme = Blockly.Theme.defineTheme('customTheme', {
      name: 'customTheme',
      base: Blockly.Themes.Classic,
      blockStyles: {
        logic_blocks: { colourPrimary: '#A55A83' },
        loop_blocks: { colourPrimary: '#3A8439' },
        math_blocks: { colourPrimary: '#3D65A8' },
        text_blocks: { colourPrimary: '#6835BB' },
        conditional_blocks: { colourPrimary: '#569BBD' },
        cycle_blocks: { colourPrimary: '#897099' },
        operations_blocks: { colourPrimary: '#B28E34' },
        variable_blocks: { colourPrimary: '#B46564' },
        procedure_blocks: { colourPrimary: '#3E7E7E' },
        text_manipulation_blocks: { colourPrimary: '#E91E63' }
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
      theme: customTheme
    });

    // Listener para eventos generales (cambios, creación y eliminación de bloques)
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
    });

    // ELIMINAR SUSCRIPTOR y PUBLICADOR
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros2_create_subscriber') ||
            xmlString.includes('ros2_minimal_publisher') ||
            xmlString.includes('ros2_create_publisher') ||
            xmlString.includes('ros2_subscriber_msg_data') ||
            xmlString.includes('ros2_publish_message')) {
            console.log('Bloque de publicador o suscriptor eliminado');
            const resultado = await this.alertService.showAlert('Acabas de eliminar un bloque de publicador o suscriptor, por ende la sesión terminará');
            console.log('El usuario presionó OK:', resultado);
            this.stopTab(tabId);
            this.consoles_sessions.delete(tabId.toString());
            this.consoles_services.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || '');
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // ELIMINAR CLIENTE
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros_create_client')) {
            console.log('Bloque de cliente eliminado');
            console.log('Bloque de publicador o suscriptor eliminado');
            //Alert para avisar que se eliminará la sesión
            const resultado = await this.alertService.showAlert('Acabas de eliminar un bloque de cliente, por ende la sesión terminará');
            console.log('El usuario presionó OK:', resultado);
            this.stopTab(tabId);
            this.consoles_sessions.delete(tabId.toString());
            this.consoles_services.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || '');
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

// ELIMINAR SERVIDOR
this.workspaces[tabId].addChangeListener(async (event) => {
  if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
    if (event.oldXml) {
      const xmlString = Blockly.Xml.domToText(event.oldXml);
      if (xmlString.includes('ros_create_server')) {
        console.log('Bloque de servidor eliminado');
        const resultado = await this.alertService.showAlert('Acabas de eliminar un bloque de servidor, por ende la sesión terminará');
        console.log('El usuario presionó OK:', resultado);
        this.stopTab(tabId);
        this.consoles_sessions.delete(tabId.toString());
        // Se asume que para el servidor el nombre del nodo se guarda en la pestaña
        const tabName = this.tabs.find(tab => tab.id === tabId)?.name || '';
        this.consoles_services.get(tabId.toString())?.deleteFile(tabName);
      }
    }
  }
  this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
});

// ELIMINAR MENSAJE
this.workspaces[tabId].addChangeListener(async (event) => {
  if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
    if (event.oldXml) {
      const xmlString = Blockly.Xml.domToText(event.oldXml);
      if (xmlString.includes('ros2_message_block')) {
        console.log('Bloque de mensaje eliminado');
        const resultado = await this.alertService.showAlert('Acabas de eliminar un bloque de mensaje, este se eliminará definitivamente');
        console.log('El usuario presionó OK:', resultado);
        this.stopTab(tabId);
        // Extraer el nombre del mensaje desde el XML; se asume que el campo se llama "MESSAGE_NAME"
        const parser = new DOMParser();
        const xmlDoc = parser.parseFromString(xmlString, "text/xml");
        const messageName = xmlDoc.querySelector('field[name="MESSAGE_NAME"]')?.textContent || '';
        console.log('Nombre del mensaje extraído:', messageName);
        this.codeService.deleteInterfaceFile('msg', messageName)
          .subscribe({
            next: (response) => console.log("Eliminado con éxito (mensaje):", response),
            error: (error) => console.error("Error al eliminar la interfaz (mensaje):", error)
          });
      }
    }
  }
  this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
});

// ELIMINAR SERVICIO
this.workspaces[tabId].addChangeListener(async (event) => {
  if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
    if (event.oldXml) {
      const xmlString = Blockly.Xml.domToText(event.oldXml);
      if (xmlString.includes('ros2_service_block')) {
        console.log('Bloque de servicio eliminado', xmlString);
        const resultado = await this.alertService.showAlert('Acabas de eliminar un bloque de servicio, este se eliminará definitivamente');
        console.log('El usuario presionó OK:', resultado);
        this.stopTab(tabId);
        // Extraer el nombre del servicio desde el XML; se asume que el campo se llama "SERVICE_NAME"
        const parser = new DOMParser();
        const xmlDoc = parser.parseFromString(xmlString, "text/xml");
        const serviceName = xmlDoc.querySelector('field[name="SERVICE_NAME"]')?.textContent || '';
        console.log('Nombre del servicio extraído:', serviceName);
        this.codeService.deleteInterfaceFile('srv', serviceName)
          .subscribe({
            next: (response) => console.log("Eliminado con éxito (servicio):", response),
            error: (error) => console.error("Error al eliminar la interfaz (servicio):", error)
          });
      }
    }
  }
  this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
});



    // Registrar el listener de eliminación de SERVIDOR para esta workspace
    //this.registerServerDeleteListenerForWorkspace(tabId);

    // Crear consola de salida y código para la pestaña
    this.consoles_output.set(tabId.toString(), '');
    this.text_code.set(tabId.toString(), '');
  }

  async addTab() {
    if (this.tabs.length >= this.MAX_NUM_PESTANAS) {
      const resultado = await this.alertService.showAlert('No se pueden agregar más de ' + this.MAX_NUM_PESTANAS + ' pestañas.');
      return;
    }
    this.updateSrvList();
    const newTabId = Date.now();
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
    do {
      newTabName = sanitizePythonFilename(`${baseName}_${index}`).replace(/\.py$/, "");
      index++;
    } while (this.tabs.some(tab => tab.name === newTabName));
    return newTabName;
  }

  selectTab(tabId: number) {
    this.selectedTabId = tabId;
    setTimeout(() => {
      this.initializeBlockly(tabId);
    }, 0);
    this.codigo_testeo_backend = this.text_code.get(tabId.toString()) || '';
    this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) || '';
  }

  storePreviousName(tab: any) {
    this.previousNames.set(tab.id, tab.name);
  }

  async changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;
    const previousName = this.previousNames.get(tabId) || tab.name;
    let sanitizedNewName = sanitizePythonFilename(newName).replace(/\.py$/, "");
    if (!sanitizedNewName) {
      const resultado = await this.alertService.showAlert('El nombre de la pestaña no puede estar vacío.');
      tab.name = previousName;
      return;
    }
    if (this.tabs.some(t => t.name === sanitizedNewName && t.id !== tabId)) {
      const resultado = await this.alertService.showAlert('Ya existe una pestaña con ese nombre.');
      tab.name = previousName;
      return;
    }
    tab.name = sanitizedNewName;
  }

  playTab(tabId: number, playAllTabs: boolean) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.text_code.set(tabId.toString(), pythonGenerator.workspaceToCode(this.workspaces[tabId]));
    }
    this.updateSrvList();
    tab.isPlaying = playAllTabs ? true : !tab.isPlaying;
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
      this.workspaces[tabId].dispose();
      delete this.workspaces[tabId];
      this.consoles_output.delete(tabId.toString());
      this.consoles_sessions.delete(tabId.toString());
      this.consoles_services.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || '');
      this.consoles_services.delete(tabId.toString());
      this.websockets.delete(tabId.toString());
      this.text_code.delete(tabId.toString());
    }
    this.tabs = this.tabs.filter(tab => tab.id !== tabId);
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
    const filteredToolbox = {
      kind: 'categoryToolbox',
      contents: this.toolbox.contents
        .map((category: any) => {
          const filteredContents = category.contents.filter((block: any) =>
            block.type.toLowerCase().includes(query)
          );
          return filteredContents.length > 0 ? { ...category, contents: filteredContents } : null;
        })
        .filter((category: any) => category !== null),
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
      this.playTab(tabId, true);
    }
  }

  stopAllTabs() {
    for (const tab of this.tabs) {
      const tabId = tab.id;
      if (!this.workspaces[tabId]) continue;
      this.stopTab(tabId);
    }
  }

  cleanConsole() {
    if (this.current_displayed_console_output !== '' && this.selectedTabId) {
      this.consoles_output.set(this.selectedTabId.toString(), '');
      this.current_displayed_console_output = 'Consola limpia';
    }
  }

  enviarCodigo(code_to_send: string, tabId: number) {
    console.log('Enviando código...');
    const workspace = this.workspaces[tabId];
    if (!workspace) {
      console.error('No existe la workspace para el tab', tabId);
      return;
    }
    let code = '';
    let fileName = '';
    let type = '';
    let serverType = '';
    const { firstLine, remainingText } = extractFirstLine(code_to_send);
    if (firstLine.indexOf('|') !== -1) {
      const parts = firstLine.split('|');
      type = parts[0];
      serverType = parts[1];
    } else {
      type = firstLine;
    }
    code = remainingText;
    if (type === "pub_sub") {
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Nodo');
      code = create_publisher(code, fileName);
    } else if (type === "server") {
      console.log('Creando servidor...');
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Servidor');
      code = create_server(code, fileName, serverType);
    } else if (type === "srv") {
      fileName = sanitizeSrvFilename(extractServiceFilename(code) || 'Servicio.srv');
      code = replaceServiceFilename(code, fileName);
    } else if (type === "msg") {
      fileName = sanitizeMsgFilename(extractMessageFilename(code) || 'FailedMsg.msg');
      code = replaceMessageFilename(code, fileName);
    } else if (type === "client") {
      console.log('Creando cliente...');
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Cliente');
      code = create_client(linesBeforeComment(code), fileName, linesAfter(code), serverType);
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
              if (this.selectedTabId === tabId) {
                this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) ?? '';
              }
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
              return of(null);
            } else if (type === "msg") {
              console.log("El archivo es un mensaje (.msg), deteniendo ejecución después de uploadCode.");
              const confirmationMessage = `Mensaje ${fileName} creado correctamente.`;
              this.consoles_output.set(tabId.toString(),
                (this.consoles_output.get(tabId.toString()) ?? '') + confirmationMessage + '\n');
              if (this.selectedTabId === tabId) {
                this.current_displayed_console_output = this.consoles_output.get(tabId.toString()) ?? '';
              }
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
              return of(null);
            }
            return codeService.executeCode(fileName);
          }),
          switchMap((response) => {
            if (!response) return of(null);
            console.log('Respuesta del backend:', response);
            const sessionId = response.session_id;
            this.consoles_sessions.set(tabId.toString(), sessionId);
            console.log('Session ID:', sessionId);
            return codeService.connectToWebSocket(sessionId);
          })
        )
        .subscribe({
          next: (response) => {
            if (!response) return;
            console.log('Mensaje WebSocket:', response.output);
            if (response.output !== this.consoles_output.get(tabId.toString())) {
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
      this.deleteTab(tabId);
    }
  }

  imageSrc: string | ArrayBuffer | null = null;

  onFileSelected(event: Event): void {
    const target = event.target as HTMLInputElement;
    if (target.files && target.files[0]) {
      const file = target.files[0];
      const reader = new FileReader();
      reader.onload = () => {
        this.imageSrc = reader.result;
      };
      reader.readAsDataURL(file);
    }
  }

  getConsoleLines(): string[] {
    return this.current_displayed_console_output
      ? this.current_displayed_console_output.trimEnd().split('\n')
      : [];
  }

  private createSrvVariableBlock(variable: any, section: string): any {
    return {
      kind: 'block',
      type: 'srv_variable',
      fields: {
        VAR_SECTION: section,
        VAR_NAME: variable.name,
        VAR_TYPE: variable.type
      }
    };
  }

  updateSrvVariablesCategory(): void {
    const toolboxObj = this.toolbox.contents && this.toolbox.contents.length > 0
      ? { ...this.toolbox }
      : { kind: 'categoryToolbox', contents: [] };

    const srvVariablesCategory = {
      kind: 'category',
      type: 'category',
      name: 'Variables de Servicio',
      contents: srvList.map((service: SrvInfo) => {
        const requestBlocks = service.variables?.request?.map((variable: any) =>
          this.createSrvVariableBlock(variable, "request")
        ) || [];
        const responseBlocks = service.variables?.response?.map((variable: any) =>
          this.createSrvVariableBlock(variable, "response")
        ) || [];
        const responseAssignBlock = {
          kind: 'block',
          type: 'srv_response_set_field',
          fields: {
            FIELD_NAME: "campo"
          }
        };
        return {
          kind: 'category',
          type: 'category',
          name: service.name ? service.name.replace(/\.srv$/, "") : "",
          contents: [
            { kind: 'label', text: "Solicitud:" },
            ...requestBlocks,
            { kind: 'label', text: "Respuesta:" },
            ...responseBlocks,
            { kind: 'label', text: "Asignar campo de respuesta:" },
            responseAssignBlock
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

  updateSrvList(): void {
    this.codeService.checkSrvFiles().subscribe(response => {
      if (response.exists) {
        srvList.length = 0;
        response.files.forEach((file: any) => {
          srvList.push(file);
        });
      } else {
        srvList.length = 0;
      }
      console.log("srvList actualizada:", srvList);
      this.updateSrvVariablesCategory();
    }, error => {
      console.error("Error al obtener la lista de archivos srv:", error);
      srvList.length = 0;
    });
  }
}

export function linesBeforeComment(code: string): string {
  const marker = "#main-sendrequest";
  const index = code.indexOf(marker);
  if (index === -1) {
    return code.trimEnd();
  }
  return code.substring(0, index).trimEnd();
}

export function linesAfter(code: string): string {
  const marker = "#main-sendrequest";
  const index = code.indexOf(marker);
  if (index === -1) {
    return "";
  }
  return code.substring(index + marker.length).trimStart();
}
