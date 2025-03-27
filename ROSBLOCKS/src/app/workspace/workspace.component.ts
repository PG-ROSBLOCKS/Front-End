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
import { extractFirstLine, reorderCodeBelowFirstMarker, extractServiceFilename, replaceSelfWithNodeInMain, linesAfter, linesBeforeComment, replaceServiceFilename, sanitizePythonFilename, sanitizeSrvFilename, sanitizeMsgFilename, extractMessageFilename, replaceMessageFilename, removeSelfInMain, sanitizeGlobalVariables, removeOneIndentLevel } from '../utilities/sanitizer-tools';
import { create_client, create_publisher, create_server } from '../blocks/code-generator';
import { srvList, SrvInfo } from '../shared/srv-list';
import { msgList, MsgInfo } from '../shared/msg-list';
import { DomSanitizer, SafeResourceUrl } from '@angular/platform-browser';
import { toolbox, updateDynamicCategoryInToolbox } from "./blockly";
import { SuccessService } from '../shared/components/success/success.service';
import { paintMap, isValidMap } from '../maps/mapBuilder';
import { map1, map2, map3 } from '../maps/maps';
import { principalBlocks } from './principal-blocks';
import { initializeCommonMsgs } from '../blocks/ros2-msgs';

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
    event.returnValue = 'Are you sure you want to leave?';
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

  tabRightClick: number | null = null;

  constructor(
    private http: HttpClient,
    private codeService: CodeService,
    private alertService: AlertService,
    private successService: SuccessService,
    private sanitizer: DomSanitizer

  ) { }

  ngOnInit(): void {
    this.reloadTurtlesim();
    this.loadFromLocalStorage();
    initializeCommonMsgs();
  }

  reloadTurtlesim(): void {
    const url = this.codeService.vncTurtlesim();
    //console.log(url);


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

  onRightClickTab(event: MouseEvent, id:number): void {
    this.selectTab(id)
    event.preventDefault();
    if(this.selectedTabId != this.tabRightClick) {
      this.tabRightClick = this.selectedTabId
    }
    else {
      this.tabRightClick = null;
    }
  }

  async duplicate(name: string, id: number): Promise<void> {
    const tempTabs = [...this.tabs];
  
    const originalWorkspace = this.workspaces[id];
    let originalXml: Element | null = null;
    if (originalWorkspace) {
      originalXml = Blockly.Xml.workspaceToDom(originalWorkspace);
    }
  
    await this.addTab();
  
    const newTabs = this.tabs.filter(tab => !tempTabs.some(t => t.id === tab.id));
    if (newTabs.length === 0) {
      console.error("No se pudo crear una nueva pestaña para duplicar.");
      return;
    }
    const newTab = newTabs[0];
  
    await this.changeTabName(newTab.id, name + '_copy');
  
    setTimeout(() => {
      const newWorkspace = this.workspaces[newTab.id];
      if (originalXml && newWorkspace) {
        Blockly.Xml.domToWorkspace(originalXml, newWorkspace);
      } else {
        console.error("No se pudo duplicar el workspace: ", { originalXml, newWorkspace });
      }
    }, 50);
  }
  

  // Escucha el click izquierdo en todo el documento
  @HostListener('document:click', ['$event'])
  onDocumentClick(event: MouseEvent): void {
    this.tabRightClick = null;
  }
  
  // Escucha el click derecho en todo el documento
  @HostListener('document:contextmenu', ['$event'])
  onDocumentContextMenu(event: MouseEvent): void {
    //This line prevents opening console on navigator
    //event.preventDefault();
  }

  saveToFile() {
    try {
      const tabsData = this.tabs.map(tab => {
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
      a.download = 'project.rosblocks';
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
    //Turns off the node if selected
    if (this.selectedTabId) {
      this.stopTab(this.selectedTabId);
    }
    
    try {
      const tabsData = this.tabs.map(tab => {
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
      
      this.enviarCodigoMapa(code);
    }
  }

  enviarCodigoMapa(code_to_send: string): void {
    //Why count to 2?, because 2 turtles are painting the map, so this indicates when a turtle ends his job
    let count = 2;
    const fileName = "turtleMap.py";
    const type = "pub_sub";
    const code = code_to_send;
  
    if (!this.mapCodeService) {
      this.mapCodeService = new CodeService(this.http);
    }
    const codeService = this.mapCodeService;
  
    this.mapFullyLoaded = false
    codeService.uploadCode(fileName, code, type)
      .pipe(
        switchMap(() => {
          return codeService.executeCode(fileName);
        }),
        switchMap((response) => {
          if (!response) return of(null);
          this.mapFullyLoaded  =false
          const sessionId = response.session_id;
          return codeService.connectToWebSocket(sessionId);
        })
      )
      .subscribe({
        next: (response) => {
          if (!response) return;
          count--
          if (count == 0) {
            this.mapFullyLoaded = true
          }
        },
        error: (error) => {
          console.error('Error:', error)
          this.mapFullyLoaded = true
        },
        complete: () => {
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
      theme: customTheme,
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
    if (tab.isPlaying) {
      this.showAlert("Canonot rename while node is playing","error");
      return;
    }

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

    const ws = this.workspaces[tabId];
    if (!ws) return;

    // 1. Validar bloques antes de ejecutar
    const topBlocks = ws.getTopBlocks(true);
    for (const block of topBlocks) {
      if (block.type === 'ros2_create_publisher') {
        const mainInput = block.getInput('MAIN');
        const childBlock = mainInput?.connection?.targetBlock();
        const hasPublisher = hasValidChain(childBlock ?? null, "ros2_publish_message");
        if (!hasPublisher) {
          this.alertService.showAlert('Error: Block "Create Publisher" needs at least one "Publish Message" inside.');
          return;
        }
      }
      if (block.type === 'ros_create_client') {
        const mainInput = block.getInput('MAIN');
        const childBlock = mainInput?.connection?.targetBlock();
        const hasClient = hasValidChain(childBlock ?? null, "ros_send_request");
        const serviceType = block.getFieldValue('CLIENT_TYPE');
        if (serviceType === ""){
          this.alertService.showAlert('Error: "Create Client" block requires a valid service.');
          return;
        }
        if (!hasClient) {
          this.alertService.showAlert('Error: "Create Client" block requires at least one valid "Send request" inside it.');
          return;
        }
      
        // 🔍 Verificar que el ros_send_request tenga TODOS los campos conectados
        let current = childBlock;
        while (current) {
          if (current.type === 'ros_send_request') {
            const allFieldsConnected = hasAllFieldsConnected(current);
            if (!allFieldsConnected) {
              this.alertService.showAlert('Error: Block "Send request" has incomplete fields.');
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
          this.alertService.showAlert('Error: Block "Create Client" needs a valid service.');
          return;
        }
      }
    }

    // 2. Continuar con lógica original
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.textCode.set(tabId.toString(), pythonGenerator.workspaceToCode(ws));
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
      //console.log('Session ID stop:', session_id);
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
        //console.log(`Tab ${tabId} was previously executed, cleaning up associated resources in backend`);
        this.stopTab(tabId);
        
        // Get all blocks from workspace
        
        const blocks = this.workspaces[tabId].getAllBlocks();
        
        // Simulate deletion of each principal block to execute cleanup logic
        blocks.forEach(block => {
          if (principalBlocks.includes(block.type)) {
            //console.log(`Processing deletion of block ${block.type} in tab ${tabId}`);
            
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

  seeTopics(): void {
    if (!this.selectedTabId) {
      console.error("No tab selected");
      return;
    }
  
    const tabId = this.selectedTabId.toString();
    const consoleService = this.consolesServices.get(tabId);
    
    if (!consoleService) {
      console.error("Console service not found for tab", tabId);
      return;
    }
  
    consoleService.seeTopics().subscribe({
      next: (topics: string[]) => {
        const topicsOutput = topics.join('\n');
        const currentOutput = this.consolesOutput.get(tabId) || '';
        
        this.consolesOutput.set(tabId, `${currentOutput}Available topics:\n${topicsOutput}\n\n`);
        this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId) || '';
      },
      error: (err) => {
        console.error("Error fetching topics:", err);
        const currentOutput = this.consolesOutput.get(tabId) || '';
        this.consolesOutput.set(tabId, `${currentOutput}Error: ${err.message}\n`);
        this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId) || '';
      }
    });
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
              const confirmationMessage = `Servicio ${fileName} created successfully.`;
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
              const confirmationMessage = `Mensaje ${fileName} created successfully.`;
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
      name: 'Service Variables',
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
            FIELD_NAME: "field"
          }
        };
        return {
          kind: 'category',
          type: 'category',
          name: service.name ? service.name.replace(/\.srv$/, "") : "",
          contents: [
            { kind: 'label', text: "Request:" },
            ...requestBlocks.map(block => {
              block.data = service.name; 
              return block;
            }),
            { kind: 'label', text: "Response:" },
            ...responseBlocks.map(block => {
              block.data = service.name;
              return block;
            }),
            responseAssignBlock
          ]
        };
      })
    };
    updateDynamicCategoryInToolbox(toolboxObj, 'ROS2 Blocks', 'Variables',  'Service Variables', srvVariablesCategory);

    // Update the toolbox of the current workspace (if active)
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.workspaces[this.selectedTabId].updateToolbox({
        kind: 'categoryToolbox',
        contents: toolboxObj.contents
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
      name: 'Message Variables',
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
              return parts[2]; // E.g., 'Twist'
            }
            return message.name; // E.g., 'message3'
          })(),
          contents: [
            { kind: 'label', text: "Message fields:" },
            ...fieldBlocks
          ]
        };
      })
    };

    updateDynamicCategoryInToolbox(toolboxObj, 'ROS2 Blocks', 'Variables', 'Message Variables', msgVariablesCategory);
    
    // Update the toolbox of the current workspace (if active)
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.workspaces[this.selectedTabId].updateToolbox({
        kind: 'categoryToolbox',
        contents: toolboxObj.contents
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
      { types: ['ros2_create_subscriber'], alertMsg: 'You have just deleted a subscriber block, therefore the session will end.', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_minimal_publisher'], alertMsg: 'You have just deleted a publisher block, therefore the session will end.', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_create_publisher'], alertMsg: 'You have just deleted a publisher block, therefore the session will end.', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_publish_message'], alertMsg: 'You have just deleted a publisher block, therefore the session will end.', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros_create_client'], alertMsg: 'You have just deleted a client block, therefore the session will end.', callback: commonDeletion, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros_create_server'], alertMsg: 'You have just removed a server block, therefore the session will end.', callback: () => {
            const tabName = this.tabs.find(t => t.id === tabId)?.name || '';
            this.stopTab(tabId);
            this.consolesSessions.delete(tabId.toString());
            this.consolesServices.get(tabId.toString())?.deleteFile(tabName);
        }, needConfirmation: false, requirePlayedCheck: true },
      { types: ['ros2_message_block'], alertMsg: 'Are you sure you want to delete this message? This action cannot be undone.', callback: (xml?: string) => {
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
        alertMsg: 'Are you sure you want to delete this service? This action will cascade all blocks associated with its variables.', 
        callback: (xml?: string) => {
          const parser = new DOMParser();
          const xmlDoc = parser.parseFromString(xml || '', "text/xml");
          const serviceName = xmlDoc.querySelector('field[name="SERVICE_NAME"]')?.textContent || '';
          console.log('Trying to remove service:', serviceName);
          this.stopTab(tabId);
          
          // Complete service information is searched in srvList
          const serviceInfo = srvList.find(s =>
            s.name === serviceName ||
            s.name === serviceName + '.srv' ||
            (s.name && s.name.replace('.srv', '') === serviceName)
          );
          
          // If the information was obtained, the associated blocks are deleted in cascade.
          if (serviceInfo && serviceInfo.variables) {
            this.cascadeDeleteServiceVariables(serviceName, serviceInfo.variables);
          } else {
            // If the information was not found, an alternative function can be called.
            // that attempts to remove blocks based only on the name
            this.globalServiceBlockDeleted(serviceName);
          }
          
          // Then proceed to delete the service in the backend
          this.codeService.deleteInterfaceFile('srv', serviceName)
            .subscribe({
              next: (response) => {
                console.log("Successfully deleted (service):", response);
                console.log('srvList before deleting:', JSON.stringify(srvList));
                const index = srvList.findIndex(s =>
                  s.name === serviceName ||
                  s.name === serviceName + '.srv' ||
                  (s.name && s.name.replace('.srv', '') === serviceName)
                );
                console.log('Index found:', index);
                if (index !== -1) {
                  srvList.splice(index, 1);
                  console.log('srvList after deleting:', JSON.stringify(srvList));
                  // Update the toolbox immediately
                  setTimeout(() => {
                    this.updateSrvVariablesCategory();
                    if (this.workspaces[tabId]) {
                      this.workspaces[tabId].refreshToolboxSelection();
                    }
                  }, 100);
                } else {
                  console.warn('Service not found in srvList:', serviceName);
                  this.updateSrvList();
                }
              },
              error: (error) => {
                console.error("Error deleting interface (service):", error);
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

  // Add this new feature to handle cascading deletion of blocks associated with a service
  globalServiceBlockDeleted(serviceName: string): void {
    console.log(`Searching for blocks dependent on the deleted service: ${serviceName}`);
    
    // Service name without extension for more flexible comparisons
    const normalizedServiceName = serviceName.replace(/\.srv$/, "");
    
    // Cycle through all tabs/workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      const allBlocks = workspace.getAllBlocks();
      
      console.log(`Reviewing tab ${tabKey} for blocks associated with the service ${normalizedServiceName}:`, allBlocks.length);
      
      // Identify all blocks that need to be removed
      const blocksToRemove: any[] = [];
      
      allBlocks.forEach((block: any) => {
        try {
          // 1. Service variables (request/response)
          if (block.type === 'srv_variable') {
            // Check if the block belongs to the service through its fields
            const variableName = block.getFieldValue('VAR_NAME');
            const variableSection = block.getFieldValue('VAR_SECTION'); // 'request' or 'response'
            
            // Find if this variable belongs to the service being removed
            const serviceInfo = srvList.find(s => 
              (s.name === normalizedServiceName || s.name === normalizedServiceName + '.srv' || s.name.replace('.srv', '') === normalizedServiceName)
            );
            
            if (serviceInfo) {
              const isInService = (variableSection === 'request' && serviceInfo.variables?.request?.some(v => v.name === variableName)) ||
                                 (variableSection === 'response' && serviceInfo.variables?.response?.some(v => v.name === variableName));
              
              if (isInService) {
                console.log(`Variable block found ${variableName} (${variableSection}) on service ${normalizedServiceName}`);
                blocksToRemove.push(block);
              }
            }
          }
          
          // 2. Response assignment blocks
          else if (block.type === 'srv_response_set_field') {
            // For srv_response_set_field, we need to check the context or some specific field
            const fieldName = block.getFieldValue('FIELD_NAME');
            
            // Estos bloques suelen estar conectados a un bloque de cliente del servicio
            // o en un contexto de uso del servicio
            let isRelatedToService = false;
            
            // Search in block connections
            if (block.parentBlock_ && block.parentBlock_.type === 'ros_create_client') {
              const clientServiceName = block.parentBlock_.getFieldValue('SERVICE_NAME');
              if (clientServiceName === normalizedServiceName) {
                isRelatedToService = true;
              }
            }
            
            // Also check if the field corresponds to a service variable
            const serviceInfo = srvList.find(s => 
              (s.name === normalizedServiceName || s.name === normalizedServiceName + '.srv' || s.name.replace('.srv', '') === normalizedServiceName)
            );
            
            if (serviceInfo && serviceInfo.variables?.response?.some(v => v.name === fieldName)) {
              isRelatedToService = true;
            }
            
            if (isRelatedToService) {
              console.log(`Found response assignment block for field ${fieldName} of service ${normalizedServiceName}`);
              blocksToRemove.push(block);
            }
          }
          
          // 3. Client blocks that use the service
          else if (block.type === 'ros_create_client') {
            const clientServiceName = block.getFieldValue('SERVICE_NAME');
            if (clientServiceName === normalizedServiceName) {
              console.log(`Client block found using service ${normalizedServiceName}`);
              blocksToRemove.push(block);
            }
          }
        } catch (e) {
          console.error(`Error processing block ${block.type}:`, e);
        }
      });
      
      // Delete all identified blocks, with their dependents
      blocksToRemove.forEach(block => {
        try {
          console.log(`Deleting block ${block.id} of type ${block.type} on tab ${tabKey}`);
          block.dispose(true); // true para eliminar bloques conectados
        } catch (e) {
          console.error("Error deleting block:", e);
        }
      });
    });
  }

  cascadeDeleteServiceVariables(serviceName: string, serviceVariables: { request?: any[]; response?: any[]; }): void {
    const normalizedServiceName = serviceName.replace(/\.srv$/, "");
    // Extract specific names from variables defined in the service
    const requestNames = serviceVariables?.request ? serviceVariables.request.map(v => v.name) : [];
    const responseNames = serviceVariables?.response ? serviceVariables.response.map(v => v.name) : [];
    console.log(`Cascading Variable Elimination for Service ${normalizedServiceName}`);
    console.log("Request variables:", requestNames, "Response variables:", responseNames);
    
    // Browse all workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      workspace.getAllBlocks().forEach((block: any) => {
        try {
          // Caso 1: Variable blocks (tipo "srv_variable")
          if (block.type === 'srv_variable') {
            const varName = block.getFieldValue('VAR_NAME');
            const varSection = block.getFieldValue('VAR_SECTION');  // "request" or "response"
            if ((varSection === 'request' && requestNames.includes(varName)) ||
                (varSection === 'response' && responseNames.includes(varName))) {
              console.log(`Deleting srv_variable block ${block.id} (${varName}, ${varSection}) on tab ${tabKey}`);
              block.dispose(true);
            }
          }
          // Caso 2: Blocks to assign response field (type "srv_response_set_field")
          else if (block.type === 'srv_response_set_field') {
            const fieldName = block.getFieldValue('FIELD_NAME');
            if (responseNames.includes(fieldName)) {
              console.log(`Removing srv_response_set_field block ${block.id} (field: ${fieldName}) on tab ${tabKey}`);
              block.dispose(true);
            }
          }
          // Caso 3 (opcional): Client blocks that use this service
          else if (block.type === 'ros_create_client') {
            const clientServiceName = block.getFieldValue('SERVICE_NAME');
            if (clientServiceName === normalizedServiceName) {
              console.log(`Removing ros_create_client block${block.id} using service ${normalizedServiceName} on tab ${tabKey}`);
              block.dispose(true);
            }
          }
        } catch (e) {
          console.error(`Error processing block ${block.type} on tab ${tabKey}:`, e);
        }
      });
    });
  }
}

export function hasValidChain(block: Blockly.Block | null, childBlock: string): boolean {
  if (!block) return false;

  if (block.type === childBlock) {
    // Validate that you have at least one field connected (extended)
    for (const input of block.inputList) {
      if (
        input.name?.startsWith('FIELD_') &&
        input.connection &&
        input.connection.targetBlock()
      ) {
        return true; // At least one field of the message is connected
      }
    }

    return false; // Has the block, but it is not extended
  }

  // Check inputs recursively
  for (const input of block.inputList) {
    const child = input.connection?.targetBlock();
    if (hasValidChain(child ?? null, childBlock)) {
      return true;
    }
  }

  // Also check out the next block connected in chain
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

  // Verify that all inputs have a block connected
  return fieldInputs.every(input => input.connection?.targetBlock());
}


