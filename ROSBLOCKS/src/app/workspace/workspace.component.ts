import { AlertService } from './../shared/components/alert/alert.service';
import { Component, AfterViewInit, OnInit, OnDestroy, ElementRef, ViewChild, HostListener } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import { definirBloquesROS2 } from '../blocks/ros2-blocks';
import { definirGeneradoresROS2 } from '../blocks/ros2-blocks-code';
import { CodeService } from '../services/code.service';
import { Subscription, of } from 'rxjs';
import { switchMap } from 'rxjs/operators';
import { extractFirstLine, reorderCodeBelowFirstMarker, extractServiceFilename, replaceSelfWithNodeInMain, replaceServiceFilename, sanitizePythonFilename, sanitizeSrvFilename, sanitizeMsgFilename, extractMessageFilename, replaceMessageFilename, removeSelfInMain, sanitizeGlobalVariables, removeOneIndentLevel } from '../utilities/sanitizer-tools';
import { create_client, create_publisher, create_server } from '../blocks/code-generator';
import { srvList, SrvInfo } from '../shared/srv-list';
import { msgList, MsgInfo } from '../shared/msg-list';
import { DomSanitizer, SafeResourceUrl } from '@angular/platform-browser';
import { toolbox } from "./blockly";
import { SuccessService } from '../shared/components/success/success.service';
import { paintMap, isValidMap } from '../maps/mapBuilder';
import { map1, map2, map3 } from '../maps/maps';
import { principalBlocks } from './principal-blocks';
import { initializeCommonMsgs } from '../blocks/ros2-msgs';
9
@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements OnDestroy {
  @ViewChild('resizer') resizer!: ElementRef;
  @ViewChild('leftSection') leftSection!: ElementRef;
  @ViewChild('rightSection') rightSection!: ElementRef;
  @ViewChild('matrixCanvas') canvasRef!: ElementRef<HTMLCanvasElement>;

  @HostListener('window:beforeunload', ['$event'])
  unloadNotification(event: BeforeUnloadEvent): void {
    event.preventDefault();
    event.returnValue = '¿Estás seguro de que quieres salir?';
  }

  isResizing = false;
  private previousNames = new Map<number, string>();
  private mapCodeService?: CodeService;
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

  matrix: number[][] = [];
  matrixLoaded = false;
  mapFullyLoaded = true;
  currentMap: number = 1;
  tabsPlayed: { tabId: number; blockName: string }[] = [];  // List to manage played tabs with the block name "ros2_create_subscriber"

  constructor(
    private http: HttpClient,
    private codeService: CodeService,
    private alertService: AlertService,
    private successService: SuccessService,
    private sanitizer: DomSanitizer

  ) { }

  ) { }

  ngOnInit(): void {
    this.reloadTurtlesim();
    this.loadFromLocalStorage();
    initializeCommonMsgs();
  }

  reloadTurtlesim(): void {
    const url = this.codeService.vncTurtlesim();
    console.log(url);


    if (url) {
      this.sanitizedVncUrl = this.sanitizer.bypassSecurityTrustResourceUrl(url);
    } else {
      this.alertService.showAlert('Could not get URL');
    }
  }

  ngOnDestroy(): void {
    for (const ws in this.websockets) {
      this.websockets.get(ws)?.unsubscribe();
    }
  }

  showMessage(message: string, type: 'success' | 'error') {
    this.successService.showSuccess(message);
  }

  showAlert(message: string, type: 'success' | 'error') {
    this.alertService.showAlert(message);
  }

  saveToFile() {
    try {
      const tabsData = this.tabs.map(tab => {
        const workspaceXml = this.workspaces[tab.id]
        const workspaceXml = this.workspaces[tab.id]
          ? Blockly.Xml.workspaceToDom(this.workspaces[tab.id]).outerHTML
          : '';
        localStorage.setItem(`workspace_${tab.id}`, workspaceXml);
        localStorage.setItem(`consoleService_${tab.id}`, "true");
        return { id: tab.id, name: tab.name };
      });
      localStorage.setItem('workspace_tabs', JSON.stringify(tabsData));
      const tabsDataSaved = JSON.parse(localStorage.getItem('workspace_tabs') || '[]');
      if (tabsDataSaved.length === 0) {
        this.showAlert('No project has been saved.', 'error');
        return;
      } else {
        this.showMessage('data saved successfully.', 'success');
      }

      const blob = new Blob([this.localStorageAsJSON()], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'proyect.rosblocks';
      a.click();
      URL.revokeObjectURL(a.href);
    } catch (error) {
      this.showMessage('Error saving data.', 'error');
    }
  }

  loadFromFile(event: Event) {
    this.setToZero();
    const input = event.target as HTMLInputElement;
    if (!input.files || input.files.length === 0) return;
    const file = input.files[0];
    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const tabsData = JSON.parse(e.target?.result as string);
        this.rewriteLocalStorageFromJSON(tabsData)
        this.loadFromLocalStorage()
      } catch (error) {
        this.showAlert('Error loading data from file.', 'error');
      }
    };
    reader.readAsText(file);
    this.showMessage('Data loaded successfully.', 'success');
  }

  saveToLocalStorage() {
    try {
      const tabsData = this.tabs.map(tab => {
        const workspaceXml = this.workspaces[tab.id]
        const workspaceXml = this.workspaces[tab.id]
          ? Blockly.Xml.workspaceToDom(this.workspaces[tab.id]).outerHTML
          : '';
        localStorage.setItem(`workspace_${tab.id}`, workspaceXml);
        localStorage.setItem(`consoleService_${tab.id}`, "true");
        return { id: tab.id, name: tab.name };
      });
      localStorage.setItem('workspace_tabs', JSON.stringify(tabsData));
      const tabsDataSaved = JSON.parse(localStorage.getItem('workspace_tabs') || '[]');
      if (tabsDataSaved.length === 0) {
        return;
      }
    } catch (error) {
    }

    this.rewriteLocalStorageFromJSON(this.localStorageAsJSON());
  }

  rewriteLocalStorageFromJSON(jsonData: string): void {
    jsonData = JSON.stringify(jsonData, null, 2);
    
    try {
      const parsedData = JSON.parse(jsonData);
  
      if (typeof parsedData === 'object' && parsedData !== null) {
        localStorage.clear();
  
        for (const key in parsedData) {
          if (Object.prototype.hasOwnProperty.call(parsedData, key)) {
            localStorage.setItem(key, parsedData[key]);
          }
        }
      } else {
        console.error('The data provided is not a valid object.');
      }
    } catch (error) {
      console.error('Error parsing JSON:', error);
    }
  }

  localStorageAsJSON(): string {
    const localStorageData: { [key: string]: string | null } = {};
    const length = localStorage.length;
  
    for (let i = 0; i < length; i++) {
      const key = localStorage.key(i);
      if (key) {
        localStorageData[key] = localStorage.getItem(key);
      }
    }
  
    return JSON.stringify(localStorageData, null, 2);
  }

  loadFromLocalStorage() {
    try {
      const tabsData = JSON.parse(localStorage.getItem('workspace_tabs') || '[]');
      if (tabsData.length === 0) {
        return;
      }
      this.setToZero();


      this.tabs = tabsData;
      setTimeout(() => {
        tabsData.forEach((tab: any) => {
          this.selectTab(tab.id);
        });
      }, 100);


      tabsData.forEach((tab: any) => {
        if (localStorage.getItem(`consoleService_${tab.id}`)) {
          this.consolesServices.set(tab.id.toString(), new CodeService(this.http));
        }
      });
    } catch (error) {
      this.showAlert('Error loading data into cache.', 'error');
    }
  }



  setToZero(): void {
    this.consolesOutput = new Map();
    this.consolesSessions = new Map();
    this.consolesServices = new Map();
    this.websockets = new Map();
    this.textCode = new Map();
    this.currentDisplayedConsoleOutput = '';
    this.testingCodeBackend = '';
    this.workspaces = {};
    this.autoScrollEnabled = true;
    this.tabs = [];
    this.selectedTabId = null;
  }

  resetTurtleContainer(map?: number): void {
    //When presing restart button
    if (map) {
      this.currentMap = map
    }
    this.mapFullyLoaded = false
    this.http.post(this.codeService.vncTurtlesimReset(), {}).subscribe({
      next: (response) => {
        if (this.currentMap == 1) {
          this.mapFullyLoaded = true
        }
        else {
          this.paint()
        }
      },
      error: (error) => {
        this.mapFullyLoaded = true
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

  paint(): void {
    this.mapFullyLoaded  = false
    let code: string = '';
    switch(this.currentMap) { 
      case 1:
        code = ''
        break;
      case 2:
        code = paintMap(map2);
        break;
      case 3:
        code = paintMap(map3);
        break;
      case 4:
        code = paintMap(this.matrix);; 
        break;
      default:
        this.alertService.showAlert("Error loading map");
        return
    }
    if (code) {
      console.log(code);
      
      this.enviarCodigoMapa(code);
    }
  }

  enviarCodigoMapa(code_to_send: string): void {
    //Why count to 2?, because 2 turtles are painting the map, so this indicates when a turtle ends his job
    let count = 2;
    console.log('Sending code to map...');
    const fileName = "turtleMap.py";
    const type = "pub_sub";
    const code = code_to_send;
  
    if (!this.mapCodeService) {
      this.mapCodeService = new CodeService(this.http);
    }
    const codeService = this.mapCodeService;
  
    console.log({ fileName, code, type });
    this.mapFullyLoaded = false
    codeService.uploadCode(fileName, code, type)
      .pipe(
        switchMap(() => {
          return codeService.executeCode(fileName);
        }),
        switchMap((response) => {
          if (!response) return of(null);
          console.log('Backend request:', response);
          this.mapFullyLoaded  =false
          const sessionId = response.session_id;
          console.log('Session id:', sessionId);
          return codeService.connectToWebSocket(sessionId);
        })
      )
      .subscribe({
        next: (response) => {
          if (!response) return;
          console.log('Websocket message:', response.output);
          count--
          if (count == 0) {
            console.log("map fully loaded");
            this.mapFullyLoaded = true
          }
        },
        error: (error) => {
          console.error('Error:', error)
          this.mapFullyLoaded = true
        },
        complete: () => {
          console.log('Process ended')
          this.mapFullyLoaded = true
        }
      });
  }

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
      toolbox: toolbox,
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

    this.workspaces[tabId].addChangeListener((event) => {
      this.saveToLocalStorage();
    });
    this.registerGenericDeletionListeners(tabId);

    this.consolesOutput.set(tabId.toString(), '');
    this.textCode.set(tabId.toString(), '');

    const savedWorkspaceXml = localStorage.getItem(`workspace_${tabId}`);
    if (savedWorkspaceXml) {
      const xmlDom = Blockly.utils.xml.textToDom(savedWorkspaceXml);
      Blockly.Xml.domToWorkspace(xmlDom, this.workspaces[tabId]);
    }

    const savedService = localStorage.getItem(`consoleService_${tabId}`);
    if (savedService) {
      this.consolesServices.set(tabId.toString(), new CodeService(this.http));
    }
  }

  async addTab() {
    if (this.tabs.length >= this.maxTabs) {
      const result = await this.alertService.showAlert('Cannot add more than ' + this.maxTabs + ' tabs.');
      return;
    }
    this.updateSrvList();
    this.updateMsgList();
    const newTabId = Date.now();
    this.tabs.push({ name: this.getUniqueTabName(), id: newTabId, isPlaying: false });
    this.consolesServices.set(newTabId.toString(), new CodeService(this.http));
    setTimeout(() => {
      this.selectTab(newTabId);
    }, 0);
    this.codeService.setNoTabs(false);
    this.saveToLocalStorage();
  }

  getUniqueTabName(): string {
    let baseName = "Node";
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
    this.updateMsgList();
    this.updateSrvList();
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
      const result = await this.alertService.showAlert('The tab name cannot be empty.');
      tab.name = previousName;
      return;
    }
    if (this.tabs.some(t => t.name === sanitizedNewName && t.id !== tabId)) {
      const result = await this.alertService.showAlert('A tab with that name already exists.');
      tab.name = previousName;
      return;
    }
    tab.name = sanitizedNewName;
  }

  playTab(tabId: number, playAllTabs: boolean) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;

    const workspace = this.workspaces[tabId];
    if (!workspace) return;

    // 1. Validar bloques antes de ejecutar
    const topBlocks = workspace.getTopBlocks(true);
    for (const block of topBlocks) {
      if (block.type === 'ros2_create_publisher') {
        const mainInput = block.getInput('MAIN');
        const childBlock = mainInput?.connection?.targetBlock();
        const hasPublisher = hasValidChain(childBlock ?? null, "ros2_publish_message");
        if (!hasPublisher) {
          this.alertService.showAlert('Error: El bloque "Create Publisher" necesita al menos un "Publish Message" en su interior.');
          return;
        }
      }
      if (block.type === 'ros_create_client') {
        const mainInput = block.getInput('MAIN');
        const childBlock = mainInput?.connection?.targetBlock();
        const hasClient = hasValidChain(childBlock ?? null, "ros_send_request");
        const serviceType = block.getFieldValue('CLIENT_TYPE');
        if (serviceType === ""){
          this.alertService.showAlert('Error: El bloque "Create Client" necesita un servicio válido.');
          return;
        }
        if (!hasClient) {
          this.alertService.showAlert('Error: El bloque "Create Client" necesita al menos un "Send request" válido en su interior.');
          return;
        }
      
        // 🔍 Verificar que el ros_send_request tenga TODOS los campos conectados
        let current = childBlock;
        while (current) {
          if (current.type === 'ros_send_request') {
            const allFieldsConnected = hasAllFieldsConnected(current);
            if (!allFieldsConnected) {
              this.alertService.showAlert('Error: El bloque "Send request" tiene campos incompletos.');
              return;
            }
            break; // ya lo encontramos y validamos
          }
          current = current.nextConnection?.targetBlock() ?? null;
        }
      }
      if (block.type === 'ros_create_server') {
        const serviceType = block.getFieldValue('SERVER_TYPE');
        if (serviceType === "") {
          this.alertService.showAlert('Error: El bloque "Create Client" necesita un servicio válido.');
          return;
        }
      }
    }

    // 2. Continuar con lógica original
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.textCode.set(tabId.toString(), pythonGenerator.workspaceToCode(workspace));
    }

    this.updateSrvList();
    this.updateMsgList();
    tab.isPlaying = playAllTabs ? true : !tab.isPlaying;

    tab.isPlaying
      ? (this.executeCode(this.textCode.get(tabId.toString()) || '', tabId),
        this.codeService.setWorkspaceChanged(false))
      : this.stopTab(tabId);
    const workspace = this.workspaces[tabId];
    if (workspace) {
      const blocks = workspace.getAllBlocks();
      blocks.forEach(block => {
        if (principalBlocks.includes(block.type)) {
          this.tabsPlayed.push({ tabId: tabId, blockName: block.type });
        }
      });
    }
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
      // Check if this tab was previously executed
      const wasPlayed = this.tabsPlayed.some(entry => entry.tabId === tabId);
      
      // If it was executed, we need to clean up backend resources
      if (wasPlayed) {
        console.log(`Tab ${tabId} was previously executed, cleaning up associated resources in backend`);
        this.stopTab(tabId);
        
        // Get all blocks from workspace
        
        const blocks = this.workspaces[tabId].getAllBlocks();
        
        // Simulate deletion of each principal block to execute cleanup logic
        blocks.forEach(block => {
          if (principalBlocks.includes(block.type)) {
            console.log(`Processing deletion of block ${block.type} in tab ${tabId}`);
            
            // Build simulated XML for the block
            const blockXml = Blockly.Xml.blockToDom(block);
            const xmlString = Blockly.Xml.domToText(blockXml);
            
            // Use existing block deletion logic
            this.handleBlockDeletion(tabId, block.type, xmlString);
          }
        });
        
        // Remove this tab from the played tabs list
        this.tabsPlayed = this.tabsPlayed.filter(entry => entry.tabId !== tabId);
      }
      
      this.workspaces[tabId].dispose();
      delete this.workspaces[tabId];
      this.consolesOutput.delete(tabId.toString());
      this.consolesSessions.delete(tabId.toString());
      this.consolesServices.get(tabId.toString())?.deleteFile(tab?.name || '');
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
    this.saveToLocalStorage();
  }

  // Helper method to handle block deletion operations
  handleBlockDeletion(tabId: number, blockType: string, xmlString: string): void {
    // Common logic for deleting services and nodes
    const commonDeletion = () => {
      this.stopTab(tabId);
      this.consolesSessions.delete(tabId.toString());
      this.consolesServices.get(tabId.toString())
        ?.deleteFile(this.tabs.find(t => t.id === tabId)?.name || '');
    };

    // Handle based on block type
    switch (blockType) {
      case 'ros2_create_subscriber':
      case 'ros2_minimal_publisher':
      case 'ros2_create_publisher':
      case 'ros2_publish_message':
      case 'ros_create_client':
        // For publisher/subscriber/client blocks - common deletion pattern
        commonDeletion();
        break;
      case 'ros_create_server':
        // For server blocks - specific handling
        const tabName = this.tabs.find(t => t.id === tabId)?.name || '';
        this.stopTab(tabId);
        this.consolesSessions.delete(tabId.toString());
        this.consolesServices.get(tabId.toString())?.deleteFile(tabName);
        break;
      case 'ros2_message_block':
        // For message interface blocks - extract name and delete from backend
        const parser = new DOMParser();
        const xmlDoc = parser.parseFromString(xmlString, "text/xml");
        const messageName = xmlDoc.querySelector('field[name="MESSAGE_NAME"]')?.textContent || '';
        if (messageName) {
          this.stopTab(tabId);
          this.codeService.deleteInterfaceFile('msg', messageName)
            .subscribe({
              next: (response) => console.log("Successfully deleted (message):", response),
              error: (error) => console.error("Error at deleting the interface (message):", error)
            });
        }
        break;
      case 'ros2_service_block':
        // For service interface blocks - extract name and delete from backend
        const serviceParser = new DOMParser();
        const serviceXmlDoc = serviceParser.parseFromString(xmlString, "text/xml");
        const serviceName = serviceXmlDoc.querySelector('field[name="SERVICE_NAME"]')?.textContent || '';
        if (serviceName) {
          this.stopTab(tabId);
          this.codeService.deleteInterfaceFile('srv', serviceName)
            .subscribe({
              next: (response) => console.log("Successfully deleted (service):", response),
              error: (error) => console.error("Error at deleting the interface (service):", error)
            });
        }
        break;
    }
  }

  onSearch(event: any): void {
    const query = event.target.value.toLowerCase();
    const filteredToolbox = {
      kind: 'categoryToolbox',
      contents: toolbox.contents
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
    this.resetTurtleContainer(this.currentMap);
  }

  cleanConsole() {
    if (this.currentDisplayedConsoleOutput !== '' && this.selectedTabId) {
      this.consolesOutput.set(this.selectedTabId.toString(), '');
      this.currentDisplayedConsoleOutput = 'console cleared';
    }
  }

  enviarCodigo(code_to_send: string, tabId: number) {
    console.log('Sending code...');
    const workspace = this.workspaces[tabId];

    let code = '';
    let fileName = '';
    let type = '';
    let serverType = '';
    const { firstLine, remainingText } = extractFirstLine(reorderCodeBelowFirstMarker(code_to_send));
    console.log('First line:', firstLine.trim());
    if (firstLine.indexOf('|') !== -1) {
      const parts = firstLine.split('|');
      type = parts[0].trim();
      serverType = parts[1].trim();
      console.log('Type:', type);
      console.log('Server type:', serverType);
    } else {
      type = firstLine.trim();
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
      console.log('Creating client...');
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Cliente');
      code = replaceSelfWithNodeInMain(create_client(linesBeforeComment(code), fileName, linesAfter(code), serverType));
    }
    const codeService = this.consolesServices.get(tabId.toString());


    if (codeService === undefined) {
      console.error('Service not found for the tab', tabId);
      return;
    } else {
      if (this.websockets.get(tabId.toString())) {
        this.websockets.get(tabId.toString())?.unsubscribe();
      }
      //make variables global in each "def"
      code = sanitizeGlobalVariables(code);
      console.log(code);

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

  onFileSelected(event: Event): void {
    
    const target = event.target as HTMLInputElement;
    if (target.files && target.files[0]) {
      const file = target.files[0];
      const reader = new FileReader();
      reader.onload = () => {
        const content = reader.result as string;
        this.matrix = this.parseMatrix(content);
        if (isValidMap(this.matrix)) {
          this.matrixLoaded = true;
          this.currentMap = 4
          this.resetTurtleContainer()
        }
        else {
          this.alertService.showAlert("File is not a map")
          return
        }
        setTimeout(() => {
          this.drawMatrixOnCanvas(this.canvasRef.nativeElement, this.matrix);
        });
      };
      reader.readAsText(file);
    }
  }

  parseMatrix(content: string): number[][] {
    return content.trim().split('\n').map(row =>
      row.trim().split('').map(char => (char === '1' ? 1 : 0))
    );
  }

  drawMatrixOnCanvas(canvas: HTMLCanvasElement, matrix: number[][]): void {
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const cellSize = 20;
    const rows = matrix.length;
    const cols = matrix[0].length;

    canvas.width = cols * cellSize;
    canvas.height = rows * cellSize;

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        ctx.fillStyle = matrix[y][x] === 1 ? '#000' : '#4556ff';
        ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
        ctx.strokeStyle = '#ccc';
        ctx.strokeRect(x * cellSize, y * cellSize, cellSize, cellSize);
      }
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
  private createMsgVariableBlock(variable: any): any {
    return {
      kind: 'block',
      type: 'msg_variable',
      fields: {
        VAR_NAME: variable.name,
        VAR_TYPE: variable.type
      }
    };
  }


  updateSrvVariablesCategory(): void {
    const toolboxObj = toolbox.contents && toolbox.contents.length > 0
      ? { ...toolbox }
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
  updateMsgVariablesCategory(): void {
    const toolboxObj = toolbox.contents && toolbox.contents.length > 0
      ? { ...toolbox }
      : { kind: 'categoryToolbox', contents: [] };

    const msgVariablesCategory = {
      kind: 'category',
      type: 'category',
      name: 'Variables de Mensaje',
      contents: msgList.map((message: MsgInfo) => {
        const fieldBlocks = message.fields?.map((variable: any) =>
          this.createMsgVariableBlock(variable)
        ) || [];

        return {
          kind: 'category',
          type: 'category',
          name: (() => {
            const parts = message.name.split('.');
            if (parts.length === 3 && parts[1] === 'msg') {
              return parts[2]; // Ej: 'Twist'
            }
            return message.name; // Ej: 'mensaje3'
          })(),
          contents: [
            { kind: 'label', text: "Campos del mensaje:" },
            ...fieldBlocks
          ]
        };
      })
    };

    const contents = toolboxObj.contents;
    const existingIdx = contents.findIndex((c: any) => c.name === 'Variables de Mensaje');
    if (existingIdx !== -1) {
      contents[existingIdx] = msgVariablesCategory;
    } else {
      contents.push(msgVariablesCategory);
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
  updateMsgList(): void {
    this.codeService.checkMsgFiles().subscribe(response => {
      if (response.exists) {
        response.files.forEach((file: any) => {
          const alreadyExists = msgList.some(msg => msg.name === file.name);
          if (!alreadyExists) {
            msgList.push(file);
          }
        });
      }
      console.log("msgList updated:", msgList);
      this.updateMsgVariablesCategory();
    }, error => {
      console.error("Error getting list of msg files:", error);
    });
  }



  registerGenericDeletionListeners(tabId: number): void {
    const ws = this.workspaces[tabId];
    const commonDeletion = () => {
      this.stopTab(tabId);
      this.consolesSessions.delete(tabId.toString());
      this.consolesServices.get(tabId.toString())
        ?.deleteFile(this.tabs.find(t => t.id === tabId)?.name || '');
    };

    const deletionHandlers = [
      { types: ['ros2_create_subscriber'], alertMsg: 'Acabas de eliminar un bloque de suscriptor, por ende la sesión terminará', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_minimal_publisher'], alertMsg: 'Acabas de eliminar un bloque de publicador, por ende la sesión terminará', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_create_publisher'], alertMsg: 'Acabas de eliminar un bloque de publicador, por ende la sesión terminará', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_publish_message'], alertMsg: 'Acabas de eliminar un bloque de publicador, por ende la sesión terminará', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros_create_client'], alertMsg: 'Acabas de eliminar un bloque de cliente, por ende la sesión terminará', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros_create_server'], alertMsg: 'Acabas de eliminar un bloque de servidor, por ende la sesión terminará', callback: () => {
            const tabName = this.tabs.find(t => t.id === tabId)?.name || '';
            this.stopTab(tabId);
            this.consolesSessions.delete(tabId.toString());
            this.consolesServices.get(tabId.toString())?.deleteFile(tabName);
        }, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_message_block'], alertMsg: '¿Estás seguro de que deseas eliminar este mensaje? Esta acción no se puede deshacer.', callback: (xml?: string) => {
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(xml || '', "text/xml");
            const messageName = xmlDoc.querySelector('field[name="MESSAGE_NAME"]')?.textContent || '';
            this.stopTab(tabId);
            this.codeService.deleteInterfaceFile('msg', messageName)
                .subscribe({
                  next: (response) => console.log("Successfully deleted (message):", response),
                  error: (error) => console.error("Error at deleting the interface (message):", error)
                });
        }, needConfirmation: true, requirePlayedCheck: false },
      { 
        types: ['ros2_service_block'], 
        alertMsg: '¿Estás seguro de que deseas eliminar este servicio? Esta acción eliminará en cascada todos los bloques asociados a sus variables.', 
        callback: (xml?: string) => {
          const parser = new DOMParser();
          const xmlDoc = parser.parseFromString(xml || '', "text/xml");
          const serviceName = xmlDoc.querySelector('field[name="SERVICE_NAME"]')?.textContent || '';
          console.log('Intentando eliminar servicio:', serviceName);
          this.stopTab(tabId);
          
          // Se busca la información completa del servicio en srvList
          const serviceInfo = srvList.find(s =>
            s.name === serviceName ||
            s.name === serviceName + '.srv' ||
            (s.name && s.name.replace('.srv', '') === serviceName)
          );
          
          // Si se obtuvo la info, se eliminan en cascada los bloques asociados
          if (serviceInfo && serviceInfo.variables) {
            this.cascadeDeleteServiceVariables(serviceName, serviceInfo.variables);
          } else {
            // Si no se encontró la info, se puede llamar a una función alternativa
            // que intente eliminar bloques basándose solo en el nombre
            this.globalServiceBlockDeleted(serviceName);
          }
          
          // Luego se procede a eliminar el servicio en el backend
          this.codeService.deleteInterfaceFile('srv', serviceName)
            .subscribe({
              next: (response) => {
                console.log("Successfully deleted (service):", response);
                console.log('srvList antes de eliminar:', JSON.stringify(srvList));
                const index = srvList.findIndex(s =>
                  s.name === serviceName ||
                  s.name === serviceName + '.srv' ||
                  (s.name && s.name.replace('.srv', '') === serviceName)
                );
                console.log('Índice encontrado:', index);
                if (index !== -1) {
                  srvList.splice(index, 1);
                  console.log('srvList después de eliminar:', JSON.stringify(srvList));
                  // Actualizar el toolbox inmediatamente
                  setTimeout(() => {
                    this.updateSrvVariablesCategory();
                    if (this.workspaces[tabId]) {
                      this.workspaces[tabId].refreshToolboxSelection();
                    }
                  }, 100);
                } else {
                  console.warn('No se encontró el servicio en srvList:', serviceName);
                  this.updateSrvList();
                }
              },
              error: (error) => {
                console.error("Error al eliminar la interfaz (servicio):", error);
                this.updateSrvList();
              }
            });
        }, 
        needConfirmation: true, 
        requirePlayedCheck: false 
      }
    ];

    ws.addChangeListener(async (event) => {
      if (event.type === Blockly.Events.BLOCK_DELETE && event instanceof Blockly.Events.BlockDelete && event.oldXml) {
        const xmlString = Blockly.Xml.domToText(event.oldXml);
        
        // Buscar si el bloque eliminado coincide con alguno de los handlers
        for (const handler of deletionHandlers) {
          const typeMatches = handler.types.some(type => xmlString.includes(type));
          const playedCondition = handler.requirePlayedCheck ? 
            this.tabsPlayed.some(entry => entry.tabId === tabId && handler.types.includes(entry.blockName)) : 
            true;
            
          if (typeMatches && playedCondition) {
            console.log(`${handler.types[0]} block removed`);
            
            // Si necesita confirmación, mostrar un diálogo de confirmación
            if (handler.needConfirmation) {
              const confirmed = await this.alertService.showConfirm(handler.alertMsg);
              if (confirmed) {
                // Si confirma, ejecutar el callback de eliminación
                console.log('User confirmed deletion');
                handler.callback(xmlString);
              } else {
                // Si cancela, hacer undo para recuperar el bloque
                console.log('User cancelled deletion, undoing...');
                ws.undo(false); // false para no hacer grupo de eventos
              }
            } else {
              // Para bloques sin confirmación, mostrar alerta normal
              const result = await this.alertService.showAlert(handler.alertMsg);
              console.log('User pressed OK:', result);
              handler.callback(xmlString);
            }
            break;
          }
        }
      }
      
      this.codeService.setNoBlocks(ws.getAllBlocks().length === 0);
    });
  }

  // Agregar esta nueva función para manejar la eliminación en cascada de bloques asociados a un servicio
  globalServiceBlockDeleted(serviceName: string): void {
    console.log(`Buscando bloques dependientes del servicio eliminado: ${serviceName}`);
    
    // Nombre del servicio sin extensión para comparaciones más flexibles
    const normalizedServiceName = serviceName.replace(/\.srv$/, "");
    
    // Recorrer todas las pestañas/workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      const allBlocks = workspace.getAllBlocks();
      
      console.log(`Revisando tab ${tabKey} para bloques asociados al servicio ${normalizedServiceName}:`, allBlocks.length);
      
      // Identificar todos los bloques que necesitan ser eliminados
      const blocksToRemove: any[] = [];
      
      allBlocks.forEach((block: any) => {
        try {
          // 1. Variables del servicio (request/response)
          if (block.type === 'srv_variable') {
            // Verificar si el bloque pertenece al servicio a través de sus campos
            const variableName = block.getFieldValue('VAR_NAME');
            const variableSection = block.getFieldValue('VAR_SECTION'); // 'request' o 'response'
            
            // Buscar si esta variable pertenece al servicio que se está eliminando
            const serviceInfo = srvList.find(s => 
              (s.name === normalizedServiceName || s.name === normalizedServiceName + '.srv' || s.name.replace('.srv', '') === normalizedServiceName)
            );
            
            if (serviceInfo) {
              const isInService = (variableSection === 'request' && serviceInfo.variables?.request?.some(v => v.name === variableName)) ||
                                 (variableSection === 'response' && serviceInfo.variables?.response?.some(v => v.name === variableName));
              
              if (isInService) {
                console.log(`Encontrado bloque de variable ${variableName} (${variableSection}) del servicio ${normalizedServiceName}`);
                blocksToRemove.push(block);
              }
            }
          }
          
          // 2. Bloques de asignación de respuesta
          else if (block.type === 'srv_response_set_field') {
            // Para srv_response_set_field, necesitamos verificar el contexto o algún campo específico
            const fieldName = block.getFieldValue('FIELD_NAME');
            
            // Estos bloques suelen estar conectados a un bloque de cliente del servicio
            // o en un contexto de uso del servicio
            let isRelatedToService = false;
            
            // Buscar en las conexiones del bloque
            if (block.parentBlock_ && block.parentBlock_.type === 'ros_create_client') {
              const clientServiceName = block.parentBlock_.getFieldValue('SERVICE_NAME');
              if (clientServiceName === normalizedServiceName) {
                isRelatedToService = true;
              }
            }
            
            // También revisar si el campo corresponde a una variable del servicio
            const serviceInfo = srvList.find(s => 
              (s.name === normalizedServiceName || s.name === normalizedServiceName + '.srv' || s.name.replace('.srv', '') === normalizedServiceName)
            );
            
            if (serviceInfo && serviceInfo.variables?.response?.some(v => v.name === fieldName)) {
              isRelatedToService = true;
            }
            
            if (isRelatedToService) {
              console.log(`Encontrado bloque de asignación de respuesta para campo ${fieldName} del servicio ${normalizedServiceName}`);
              blocksToRemove.push(block);
            }
          }
          
          // 3. Bloques de cliente que utilizan el servicio
          else if (block.type === 'ros_create_client') {
            const clientServiceName = block.getFieldValue('SERVICE_NAME');
            if (clientServiceName === normalizedServiceName) {
              console.log(`Encontrado bloque cliente usando servicio ${normalizedServiceName}`);
              blocksToRemove.push(block);
            }
          }
        } catch (e) {
          console.error(`Error al procesar bloque ${block.type}:`, e);
        }
      });
      
      // Eliminar todos los bloques identificados, con sus dependientes
      blocksToRemove.forEach(block => {
        try {
          console.log(`Eliminando bloque ${block.id} de tipo ${block.type} en tab ${tabKey}`);
          block.dispose(true); // true para eliminar bloques conectados
        } catch (e) {
          console.error("Error al eliminar bloque:", e);
        }
      });
    });
  }

  cascadeDeleteServiceVariables(serviceName: string, serviceVariables: { request?: any[]; response?: any[]; }): void {
    const normalizedServiceName = serviceName.replace(/\.srv$/, "");
    // Extraer nombres específicos de variables definidas en el servicio
    const requestNames = serviceVariables?.request ? serviceVariables.request.map(v => v.name) : [];
    const responseNames = serviceVariables?.response ? serviceVariables.response.map(v => v.name) : [];
    console.log(`Eliminando en cascada variables para el servicio ${normalizedServiceName}`);
    console.log("Request variables:", requestNames, "Response variables:", responseNames);
    
    // Recorrer todas las workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      workspace.getAllBlocks().forEach((block: any) => {
        try {
          // Caso 1: Bloques de variables (tipo "srv_variable")
          if (block.type === 'srv_variable') {
            const varName = block.getFieldValue('VAR_NAME');
            const varSection = block.getFieldValue('VAR_SECTION');  // "request" o "response"
            if ((varSection === 'request' && requestNames.includes(varName)) ||
                (varSection === 'response' && responseNames.includes(varName))) {
              console.log(`Eliminando bloque srv_variable ${block.id} (${varName}, ${varSection}) en tab ${tabKey}`);
              block.dispose(true);
            }
          }
          // Caso 2: Bloques para asignar campo de respuesta (tipo "srv_response_set_field")
          else if (block.type === 'srv_response_set_field') {
            const fieldName = block.getFieldValue('FIELD_NAME');
            if (responseNames.includes(fieldName)) {
              console.log(`Eliminando bloque srv_response_set_field ${block.id} (campo: ${fieldName}) en tab ${tabKey}`);
              block.dispose(true);
            }
          }
          // Caso 3 (opcional): Bloques de cliente que utilicen este servicio
          else if (block.type === 'ros_create_client') {
            const clientServiceName = block.getFieldValue('SERVICE_NAME');
            if (clientServiceName === normalizedServiceName) {
              console.log(`Eliminando bloque ros_create_client ${block.id} usando servicio ${normalizedServiceName} en tab ${tabKey}`);
              block.dispose(true);
            }
          }
        } catch (e) {
          console.error(`Error al procesar bloque ${block.type} en tab ${tabKey}:`, e);
        }
      });
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
  if (index === -1) return "";
  // Extrae todo lo que sigue al marcador, sin alterar la indentación original.
  return removeOneIndentLevel(code.substring(index + marker.length))
}

export function hasValidChain(block: Blockly.Block | null, childBlock: string): boolean {
  if (!block) return false;

  if (block.type === childBlock) {
    // Validar que tiene al menos un campo conectado (extendido)
    for (const input of block.inputList) {
      if (
        input.name?.startsWith('FIELD_') &&
        input.connection &&
        input.connection.targetBlock()
      ) {
        return true; // Al menos un campo del mensaje está conectado
      }
    }

    return false; // Tiene el bloque, pero no está extendido
  }

  // Revisa inputs recursivamente
  for (const input of block.inputList) {
    const child = input.connection?.targetBlock();
    if (hasValidChain(child ?? null, childBlock)) {
      return true;
    }
  }

  // También revisa el siguiente bloque conectado en cadena
  const next = block.nextConnection?.targetBlock();
  if (hasValidChain(next ?? null, childBlock)) {
    return true;
  }

  return false;
}

export function hasAllFieldsConnected(block: Blockly.Block): boolean {
  if (!block || block.type !== 'ros_send_request') return false;

  const fieldInputs = block.inputList.filter(input => input.name?.startsWith("FIELD_"));
  if (fieldInputs.length === 0) return false;

  // Verifica que todos los inputs tengan un bloque conectado
  return fieldInputs.every(input => input.connection?.targetBlock());
}


