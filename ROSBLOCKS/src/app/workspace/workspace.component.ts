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
import { SafeResourceUrl } from '@angular/platform-browser';
@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements OnDestroy {
  @ViewChild('resizer') resizer!: ElementRef;
  @ViewChild('leftSection') leftSection!: ElementRef;
  @ViewChild('rightSection') rightSection!: ElementRef;

  isResizing = false;
  private previousNames = new Map<number, string>();
  maxTabs = 8;
  consolesOutput: Map<string, string> = new Map();
  consolesSessions: Map<string, string> = new Map();
  consolesServices: Map<string, CodeService> = new Map();
  websockets: Map<string, Subscription> = new Map();
  textCode: Map<string, string> = new Map();
  currentDisplayedConsoleOutput: string = '';
  testingCodeBackend: string = '';
  workspaces: { [key: number]: Blockly.WorkspaceSvg } = {};
  autoScrollEnabled: boolean = true;
  tabs: { name: string; id: number; isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;
  sanitizedVncUrl!: SafeResourceUrl;

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
      },
      {
        kind: 'category',
        name: 'Turtlesim',
        contents: [
          { kind: "block", type: "ros2_publish_twist" },
          { kind: "block", type: "ros2_rotate_turtle" },
          { kind: "block", type: "ros2_turtle_set_pose" },
        ]
      }
    ],
  };

  constructor(
    private http: HttpClient,
    private codeService: CodeService,
    private alertService: AlertService,
  ) { }

  ngOnDestroy(): void {
    for (const ws in this.websockets) {
      this.websockets.get(ws)?.unsubscribe();
    }
  }

  resetTurtleContainer(): void {
    this.http.post('http://localhost:8000/reset/', {}).subscribe({
      next: (response) => {
        console.log("Turltesim restarted:", response);
      },
      error: (error) => {
        console.error("Error restarting Turtlesim:", error);
      }
    });
  }

  // END TEST AREA
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

    // Listener for general events (changes, creation & deleting blocks)
    this.workspaces[tabId].addChangeListener((event) => {
      if (event.type === Blockly.Events.BLOCK_CHANGE) {
        this.codeService.setWorkspaceChanged(true);
        console.log('Event detected: BLOCK_CHANGE. Flag updated.');
      } else if (event.type === Blockly.Events.BLOCK_CREATE) {
        this.codeService.setWorkspaceChanged(true);
        console.log('Event detected: BLOCK_CREATE. Flag updated.');
      } else if (event.type === Blockly.Events.BLOCK_DELETE) {
        this.codeService.setWorkspaceChanged(true);
        console.log('Event detected: BLOCK_DELETE. Flag updated.');
      }
    });

    // DELETE SUSCPRITOR & PUBLISHER
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros2_create_subscriber') ||
            xmlString.includes('ros2_minimal_publisher') ||
            xmlString.includes('ros2_create_publisher') ||
            xmlString.includes('ros2_subscriber_msg_data') ||
            xmlString.includes('ros2_publish_message')) {
            console.log('Publisher or subscriber block removed');
            const result = await this.alertService.showAlert('Acabas de eliminar un bloque de publicador o suscriptor, por ende la sesión terminará');
            console.log('User pressed OK:', result);
            this.stopTab(tabId);
            this.consolesSessions.delete(tabId.toString());
            this.consolesServices.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || '');
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // DELETE CLIENTE
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros_create_client')) {
            console.log('Client block deleted');
            console.log('Publisher or subscriber block removed');
            //Alert to advice session will be deleted
            const result = await this.alertService.showAlert('Acabas de eliminar un bloque de cliente, por ende la sesión terminará');
            console.log('User pressed OK:', result);
            this.stopTab(tabId);
            this.consolesSessions.delete(tabId.toString());
            this.consolesServices.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || '');
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // DELETE SERVER
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros_create_server')) {
            console.log('Server block removed');
            const result = await this.alertService.showAlert('Acabas de eliminar un bloque de servidor, por ende la sesión terminará');
            console.log('User pressed OK:', result);
            this.stopTab(tabId);
            this.consolesSessions.delete(tabId.toString());
            // It is assumed that for the server the node name is saved in the tab
            const tabName = this.tabs.find(tab => tab.id === tabId)?.name || '';
            this.consolesServices.get(tabId.toString())?.deleteFile(tabName);
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // DELETE MESSAGE
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros2_message_block')) {
            console.log('Message block removed');
            const result = await this.alertService.showAlert('Acabas de eliminar un bloque de mensaje, este se eliminará definitivamente');
            console.log('User pressed OK:', result);
            this.stopTab(tabId);
            // Extract the message name from the XML; the field is assumed to be called "MESSAGE_NAME"
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(xmlString, "text/xml");
            const messageName = xmlDoc.querySelector('field[name="MESSAGE_NAME"]')?.textContent || '';
            console.log('Message name extracted:', messageName);
            this.codeService.deleteInterfaceFile('msg', messageName)
              .subscribe({
                next: (response) => console.log("Successfully deleted (message):", response),
                error: (error) => console.error("Error at deleting the interface (message):", error)
              });
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // DELETE SERVICIO
    this.workspaces[tabId].addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete) {
        if (event.oldXml) {
          const xmlString = Blockly.Xml.domToText(event.oldXml);
          if (xmlString.includes('ros2_service_block')) {
            console.log('Service block removed', xmlString);
            const result = await this.alertService.showAlert('Acabas de eliminar un bloque de servicio, este se eliminará definitivamente');
            console.log('User pressed OK:', result);
            this.stopTab(tabId);
            // Extract the service name from the XML; the field is assumed to be called "SERVICE_NAME"
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(xmlString, "text/xml");
            const serviceName = xmlDoc.querySelector('field[name="SERVICE_NAME"]')?.textContent || '';
            console.log('Message name extracted:', serviceName);
            this.codeService.deleteInterfaceFile('srv', serviceName)
              .subscribe({
                next: (response) => console.log("Successfully deleted (service):", response),
                error: (error) => console.error("Error at deleting the interface (service):", error)
              });
          }
        }
      }
      this.codeService.setNoBlocks(this.workspaces[tabId].getAllBlocks().length === 0);
    });

    // Register the SERVER delete listener for this workspace
    //this.registerServerDeleteListenerForWorkspace(tabId);
    this.consolesOutput.set(tabId.toString(), '');
    this.textCode.set(tabId.toString(), '');
  }

  async addTab() {
    if (this.tabs.length >= this.maxTabs) {
      const result = await this.alertService.showAlert('No se pueden agregar más de ' + this.maxTabs + ' pestañas.');
      return;
    }
    this.updateSrvList();
    const newTabId = Date.now();
    this.tabs.push({ name: this.getUniqueTabName(), id: newTabId, isPlaying: false });
    this.consolesServices.set(newTabId.toString(), new CodeService(this.http));
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
    this.testingCodeBackend = this.textCode.get(tabId.toString()) || '';
    this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) || '';
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
      const result = await this.alertService.showAlert('El nombre de la pestaña no puede estar vacío.');
      tab.name = previousName;
      return;
    }
    if (this.tabs.some(t => t.name === sanitizedNewName && t.id !== tabId)) {
      const result = await this.alertService.showAlert('Ya existe una pestaña con ese nombre.');
      tab.name = previousName;
      return;
    }
    tab.name = sanitizedNewName;
  }

  playTab(tabId: number, playAllTabs: boolean) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.textCode.set(tabId.toString(), pythonGenerator.workspaceToCode(this.workspaces[tabId]));
    }
    this.updateSrvList();
    tab.isPlaying = playAllTabs ? true : !tab.isPlaying;
    tab.isPlaying
      ? (this.executeCode(this.textCode.get(tabId.toString()) || '', tabId),
        this.codeService.setWorkspaceChanged(false))
      : this.stopTab(tabId);
  }

  stopTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.isPlaying = false;
      const session_id = this.consolesSessions.get(tabId.toString());
      console.log('Session ID stop:', session_id);
      if (session_id) {
        this.consolesServices.get(tabId.toString())?.killExecution(session_id);
      }
      this.consolesServices.get(tabId.toString())?.closeConnection();
      this.websockets.get(tabId.toString())?.unsubscribe();
    }
  }

  deleteTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (this.workspaces[tabId]) {
      this.workspaces[tabId].dispose();
      delete this.workspaces[tabId];
      this.consolesOutput.delete(tabId.toString());
      this.consolesSessions.delete(tabId.toString());
      this.consolesServices.get(tabId.toString())?.deleteFile(this.tabs.find(tab => tab.id === tabId)?.name || '');
      this.consolesServices.delete(tabId.toString());
      this.websockets.delete(tabId.toString());
      this.textCode.delete(tabId.toString());
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
    this.resetTurtleContainer();
  }

  cleanConsole() {
    if (this.currentDisplayedConsoleOutput !== '' && this.selectedTabId) {
      this.consolesOutput.set(this.selectedTabId.toString(), '');
      this.currentDisplayedConsoleOutput = 'Consola limpia';
    }
  }

  enviarCodigo(code_to_send: string, tabId: number) {
    console.log('Sending code...');
    const workspace = this.workspaces[tabId];
    if (!workspace) {
      console.error('Theres no workspace for the tab', tabId);
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
      console.log('Creating server...');
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Servidor');
      code = create_server(code, fileName, serverType);
    } else if (type === "srv") {
      fileName = sanitizeSrvFilename(extractServiceFilename(code) || 'Servicio.srv');
      code = replaceServiceFilename(code, fileName);
    } else if (type === "msg") {
      fileName = sanitizeMsgFilename(extractMessageFilename(code) || 'FailedMsg.msg');
      code = replaceMessageFilename(code, fileName);
    } else if (type === "client") {
      console.log('Creanting client...');
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Cliente');
      code = create_client(linesBeforeComment(code), fileName, linesAfter(code), serverType);
    }
    const codeService = this.consolesServices.get(tabId.toString());
    if (codeService === undefined) {
      console.error('Service not found for the tab', tabId);
      return;
    } else {
      if (this.websockets.get(tabId.toString())) {
        this.websockets.get(tabId.toString())?.unsubscribe();
      }
      this.websockets.set(tabId.toString(), codeService.uploadCode(fileName, code, type)
        .pipe(
          switchMap(() => {
            if (type === "srv") {
              console.log("The file is a service (.srv), stopping execution after uploadCode.");
              const confirmationMessage = `Servicio ${fileName} creado correctamente.`;
              this.consolesOutput.set(tabId.toString(),
                (this.consolesOutput.get(tabId.toString()) ?? '') + confirmationMessage + '\n');
              if (this.selectedTabId === tabId) {
                this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) ?? '';
              }
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
              return of(null);
            } else if (type === "msg") {
              console.log("The file is a menssage (.msg), stopping execution after uploadCode.");
              const confirmationMessage = `Mensaje ${fileName} creado correctamente.`;
              this.consolesOutput.set(tabId.toString(),
                (this.consolesOutput.get(tabId.toString()) ?? '') + confirmationMessage + '\n');
              if (this.selectedTabId === tabId) {
                this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) ?? '';
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
            console.log('Backend response:', response);
            const sessionId = response.session_id;
            this.consolesSessions.set(tabId.toString(), sessionId);
            console.log('Session ID:', sessionId);
            return codeService.connectToWebSocket(sessionId);
          })
        )
        .subscribe({
          next: (response) => {
            if (!response) return;
            console.log('Websocket message:', response.output);
            if (response.output !== this.consolesOutput.get(tabId.toString())) {
              this.consolesOutput.set(tabId.toString(), (this.consolesOutput.get(tabId.toString()) ?? '') + response.output + '\n');
              if (this.selectedTabId === tabId) {
                this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) ?? '';
              }
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
            }
          },
          error: (error) => console.error('Error:', error),
          complete: () => console.log('Completed process')
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
    return this.currentDisplayedConsoleOutput
      ? this.currentDisplayedConsoleOutput.trimEnd().split('\n')
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
      console.log("srvList updated:", srvList);
      this.updateSrvVariablesCategory();
    }, error => {
      console.error("Error getting list of srv files:", error);
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
