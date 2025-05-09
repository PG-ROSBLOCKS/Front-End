import { AlertService } from './../shared/components/alert/alert.service';
import { Component, OnDestroy, ElementRef, ViewChild, HostListener, ChangeDetectorRef } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import { definirBloquesROS2, setMessageService } from '../blocks/ros2-blocks';
import { clearImports, definirGeneradoresROS2 } from '../blocks/ros2-blocks-code';
import { CodeService } from '../services/code.service';
import { Observable, Subscription, forkJoin, of } from 'rxjs';
import { filter, switchMap, take, tap } from 'rxjs/operators';
import { extractFirstLine, reorderCodeBelowFirstMarker, extractServiceFilename, replaceSelfWithNodeInMain, linesAfter, linesBeforeComment, replaceServiceFilename, sanitizePythonFilename, sanitizeSrvFilename, sanitizeMsgFilename, extractMessageFilename, replaceMessageFilename, removeSelfInMain, sanitizeGlobalVariables, removeOneIndentLevel } from '../utilities/sanitizer-tools';
import { create_client, create_publisher, create_server } from '../blocks/code-generator';
import { srvList, SrvInfo } from '../shared/srv-list';
import { msgList, MsgInfo, MsgVariable, customMsgList } from '../shared/msg-list';
import { DomSanitizer, SafeResourceUrl } from '@angular/platform-browser';
import { toolbox, updateDeepCategoryInToolbox, updateNestedCategoryInToolbox } from "./blockly";
import { SuccessService } from '../shared/components/success/success.service';
import { paintMap, isValidMap } from '../maps/mapBuilder';
import { map1, map2, map3 } from '../maps/maps';
import { principalBlocks } from './principal-blocks';
import { initializeCommonMsgs } from '../blocks/ros2-msgs';
import { blockColors } from '../blocks/color-palette';
import { MessageService } from '../shared/message.service';
import { ErrorsService } from '../shared/components/error/errors.service';
import { parseMatrix } from './workspace-utils';
import { colour } from 'blockly/blocks';


@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent implements OnDestroy {
  private lastActivityTimestamp: number = Date.now();
  private inactivityTimer: any;

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
  autoScrollEnabled: boolean = false;
  tabs: { name: string; id: number; isPlaying: boolean }[] = [];
  selectedTabId: number | null = null;
  sanitizedVncUrl!: SafeResourceUrl;

  matrix: number[][] = [];
  matrixLoaded = false;
  mapFullyLoaded = true;
  currentMap: number = 1;
  tabsPlayed: { tabId: number; blockName: string }[] = [];  // List to manage played tabs with the block name "ros2_create_subscriber"

  tabRightClick: number | null = null;
  mapSessionId: string = '';

  //test auxiliar variables
  private allTotal = 0;
  private allReady = 0;
  private allRunning = false;
  private seenMsgs = new WeakSet<any>();

  constructor(
    private http: HttpClient,
    private codeService: CodeService,
    private alertService: AlertService,
    private successService: SuccessService,
    private errorsService: ErrorsService,
    private sanitizer: DomSanitizer,
    private messageService: MessageService,
    private changeDetectorRef: ChangeDetectorRef
  ) { }

  ngOnInit(): void {
    this.reloadTurtlesim();
    this.loadFromLocalStorage();
    initializeCommonMsgs();
    setMessageService(this.messageService);
    this.blockErrorMessages();
    this.startInactivityCheck();
  }

  @HostListener('window:mousemove')
  @HostListener('window:keydown')
  @HostListener('window:click')
  @HostListener('window:scroll')
  resetInactivityTimer(): void {
    this.lastActivityTimestamp = Date.now();
    if (this.inactivityTimer) {
      clearTimeout(this.inactivityTimer);
      this.inactivityTimer = null;
    }
  }

  private startInactivityCheck(): void {
  }

  blockErrorMessages(): void {
    this.messageService.message$.subscribe((msg) => {
      if (msg.type === 'SERVICE_MISMATCH') {
        const { expected } = msg.payload;
        this.errorsService.showErrors(expected)
      }
    });
  }

  reloadTurtlesim(): void {
    const url = this.codeService.vncTurtlesim();

    if (url) {
      this.sanitizedVncUrl = this.sanitizer.bypassSecurityTrustResourceUrl(url);
    } else {
      this.alertService.showAlert('Could not get URL');
    }
  }

  ngOnDestroy(): void {
    // Unsubscribe from all active WebSocket connections
    this.websockets.forEach((subscription, tabId) => {
      console.log(`Unsubscribing WebSocket for tab ${tabId}`);
      subscription.unsubscribe();
    });
    this.websockets.clear(); // Clear the map after unsubscribing

    // Close connections for all CodeService instances associated with tabs
    this.consolesServices.forEach((service, tabId) => {
      console.log(`Closing CodeService connection for tab ${tabId}`);
      service.closeConnection(); // Ensure WebSocketSubject is completed
      const sessionId = this.consolesSessions.get(tabId);
      if (sessionId) {
        console.log(`Requesting termination for session ${sessionId} during destroy`);
        // Optionally kill any remaining backend process if the component is destroyed
        // Be cautious with this, might not always be desired
        // service.killExecution(sessionId);
      }
    });
    this.consolesServices.clear(); // Clear the map

    // Stop any inactivity timers
    if (this.inactivityTimer) {
        clearTimeout(this.inactivityTimer);
    }
    // Consider saving workspace state one last time if needed
    // this.saveToLocalStorage();
  }

  showMessage(message: string, type: 'success' | 'error') {
    this.successService.showSuccess(message);
  }

  showAlert(message: string, type: 'success' | 'error') {
    this.alertService.showAlert(message);
  }

  onRightClickTab(event: MouseEvent, id: number): void {
    this.selectTab(id)
    event.preventDefault();
    if (this.selectedTabId != this.tabRightClick) {
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
      console.error("Could not create a  new duplicated tab.");
      return;
    }
    const newTab = newTabs[0];

    await this.changeTabName(newTab.id, name + '_copy');

    setTimeout(() => {
      const newWorkspace = this.workspaces[newTab.id];
      if (originalXml && newWorkspace) {
        Blockly.Xml.domToWorkspace(originalXml, newWorkspace);
      } else {
        console.error("Could not duplicate workspace: ", { originalXml, newWorkspace });
      }
    }, 50);
  }


  // Listens teft click
  @HostListener('document:click', ['$event'])
  onDocumentClick(event: MouseEvent): void {
    this.tabRightClick = null;
  }

  // Listens right click
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
        const fileData = JSON.parse(e.target?.result as string);

        if (fileData['mapSessionId']) {
          this.mapSessionId = fileData['mapSessionId'];
        }

        this.mapCodeService = new CodeService(this.http);


        if (this.mapSessionId && this.currentMap !== 1) {
          setTimeout(() => {
            this.paint();
          }, 300);
        }

        this.rewriteLocalStorageFromJSON(fileData);

        this.loadFromLocalStorage()
      } catch (error) {
        this.showAlert('Error loading data from file.', 'error');
      }
    };
    reader.readAsText(file);
    this.resetTurtleContainer(1)
    //this.showMessage('Data loaded successfully.', 'success');
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
        localStorage.setItem('mapSessionId', this.mapSessionId);
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
        //console.error('The data provided is not a valid object.');
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
      this.setToZero(); // Mover aquí si es seguro

      // Llama a las funciones que devuelven Observables
      const srvUpdate$ = this.updateSrvList();
      const msgUpdate$ = this.updateMsgList();

      // Espera a que ambas actualizaciones terminen
      forkJoin({ srv: srvUpdate$, msg: msgUpdate$ }).subscribe({ // <-- A esta línea (object map)
        // next: ([srvResult, msgResult]) => { // <-- Cambiar esta línea
        next: () => { // <-- A e
          // --- TODA la lógica que depende de las listas y carga workspaces va AQUÍ ---
          //console.log('SrvList and MsgList updated, proceeding to load workspaces.');

          this.tabs = tabsData;

          // No necesitas setTimeout aquí, las listas ya están cargadas
          tabsData.forEach((tab: any) => {
            // Es importante que selectTab -> initializeBlockly -> domToWorkspace
            // ocurra DESPUÉS de que las listas estén listas.
            this.selectTab(tab.id);
            // O podrías llamar a initializeBlockly directamente si selectTab hace más cosas
            // setTimeout(() => this.initializeBlockly(tab.id), 0); // Podría ser necesario un pequeño delay si hay problemas de renderizado
          });


          tabsData.forEach((tab: any) => {
            if (localStorage.getItem(`consoleService_${tab.id}`)) {
              this.consolesServices.set(tab.id.toString(), new CodeService(this.http));
            }
          });

          this.mapSessionId = localStorage.getItem('mapSessionId') || '';
          this.mapCodeService = new CodeService(this.http);

          if (this.mapSessionId && this.currentMap !== 1) {
            setTimeout(() => {
              this.paint();
            }, 300); // Este setTimeout puede permanecer si es para pintar
          }
          // ... cualquier otra lógica que estuviera después de cargar tabsData ...

          // Seleccionar la primera pestaña si es necesario (después de inicializar todas)
          if (this.tabs.length > 0 && !this.selectedTabId) {
            this.selectTab(this.tabs[0].id);
          }

          this.showMessage('Last session recovered.', 'success');
        },
        error: (err) => {
          console.error('Error updating service or message lists:', err);
          this.showAlert('Error loading initial data.', 'error');
          // Aún podrías intentar cargar los workspaces aquí, pero los dropdowns podrían fallar
          this.tabs = tabsData;
          tabsData.forEach((tab: any) => this.selectTab(tab.id)); // Intenta cargar de todas formas
        }
      });
      // --- FIN DEL BLOQUE forkJoin ---

    } catch (error) {
      console.error('Error parsing local storage:', error);
      this.showAlert('Error reading saved data.', 'error');
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
    this.deleteMap()
    this.mapSessionId = ''
  }

  resetTurtleContainer(map?: number): void {
    this.deleteMap()
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
    this.mapFullyLoaded = false
    let code: string = '';
    switch (this.currentMap) {
      case 1:
        code = paintMap(map1);
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
    let count = 2;
    const fileName = "turtleMap.py";
    const type = "pub_sub";
    const code = code_to_send;

    if (!this.mapCodeService) {
      this.mapCodeService = new CodeService(this.http);
    }
    const codeService = this.mapCodeService;

    this.mapFullyLoaded = false;
    codeService.ready$.pipe(               // ← USAMOS codeService.ready$
      filter(ready => ready),
      take(1),

      // 1) cuando esté listo, subo
      switchMap(() => codeService.uploadCode(fileName, code, type)),  // ← USAMOS codeService.uploadCode

      // 2) luego ejecuto
      switchMap(() => codeService.executeCode(fileName)),             // ← USAMOS codeService.executeCode

      // 3) abro WS
      switchMap(response => {
        if (!response) return of(null);
        this.mapFullyLoaded = false;
        this.mapSessionId = response.session_id;
        return codeService.connectToWebSocket(this.mapSessionId);     // ← USAMOS codeService.connectToWebSocket
      })
    ).subscribe({
      next: wsMsg => {
        if (!wsMsg) return;
        count--;
        if (count === 0) {
          this.mapFullyLoaded = true;
          this.saveToLocalStorage();
        }
      },
      error: err => {
        console.error('Error:', err);
        this.mapFullyLoaded = true;
      },
      complete: () => {
        this.mapFullyLoaded = true;
      }
    });
  }


  deleteMap(): void {

    if (!this.mapCodeService || !this.mapSessionId) {
      return;
    }

    this.mapCodeService.killExecution(this.mapSessionId);
    this.mapCodeService.closeConnection();

    this.mapSessionId = '';
    this.mapFullyLoaded = true;
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
        logic_blocks: { colourPrimary: blockColors.Conditionals }, //Conditionals
        loop_blocks: { colourPrimary: blockColors.Cycles }, //Cycles
        math_blocks: { colourPrimary: blockColors.Operations }, //Operations
        variable_blocks: { colourPrimary: blockColors.Variables }, //Variables
        procedure_blocks: { colourPrimary: blockColors.Functions },//Functions
        text_blocks: { colourPrimary: blockColors.Text }, //Text
        conditional_blocks: { colourPrimary: blockColors.Conditionals },
        cycle_blocks: { colourPrimary: blockColors.Cycles },
        operations_blocks: { colourPrimary: blockColors.Cycles },
        text_manipulation_blocks: { colourPrimary: blockColors.Functions }
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
      if (event.type != 'viewport_change' && event.type != 'selected' && event.type != 'click') {
        this.codeService.setNoBlocks(
          this.workspaces[tabId].getAllBlocks(false).length === 0
        );
        this.codeService.setNoTabs(this.tabs.length === 0);
        this.saveToLocalStorage();
      }
      if (event.type === Blockly.Events.BLOCK_CHANGE || event.type === Blockly.Events.BLOCK_CREATE || event.type === Blockly.Events.BLOCK_DELETE) {
        this.codeService.setWorkspaceChanged(true);
      }
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
    this.updateSrvList().subscribe();
    this.updateMsgList().subscribe();
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
    this.updateMsgList().subscribe();
    this.updateSrvList().subscribe();
  }

  storePreviousName(tab: any) {
    this.previousNames.set(tab.id, tab.name);
  }

  async changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (!tab) return;
    if (tab.isPlaying) {
      this.showAlert("Canonot rename while node is playing", "error");
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

    // 1. Validates block before execution
    const topBlocks = ws.getTopBlocks(true);
    for (const block of topBlocks) {
      clearImports();
      if (block.type === 'ros2_create_publisher') {
        const mainInput = block.getInput('MAIN');
        const childBlock = mainInput?.connection?.targetBlock();
        const hasPublisher = hasValidChain(childBlock ?? null, "ros2_publish_message");
        const topic = block.getFieldValue('TOPIC_NAME');
        if (!hasPublisher) {
          this.alertService.showAlert('Error: Block "Create Publisher" needs at least one "Publish Message" inside.');
          return;
        }
        if (topic === "/") {
          this.alertService.showAlert('Error: Block "Create Publisher" needs a valid topic.');
          return;
        }
      }
      if (block.type === 'ros2_create_subscriber') {
        const topic = block.getFieldValue('TOPIC_NAME');
        if (topic === "/") {
          this.alertService.showAlert('Error: Block "Create Subscriber" needs a valid topic.');
          return;
        }
      }
      if (block.type === 'ros_create_client') {
        const mainInput = block.getInput('MAIN');
        const childBlock = mainInput?.connection?.targetBlock();
        const hasClient = hasValidChain(childBlock ?? null, "ros_send_request");
        const serviceType = block.getFieldValue('CLIENT_TYPE');
        if (serviceType === "") {
          this.alertService.showAlert('Error: "Create Client" block requires a valid service.');
          return;
        }
        if (!hasClient) {
          this.alertService.showAlert('Error: "Create Client" block requires at least one valid "Send request" inside it.');
          return;
        }

        // check ros_send_request have ALL field connected
        let current = childBlock;
        while (current) {
          if (current.type === 'ros_send_request') {
            const allFieldsConnected = hasAllFieldsConnected(current);
            if (!allFieldsConnected) {
              this.alertService.showAlert('Error: Block "Send request" has incomplete fields.');
              return;
            }
            break;
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

    // 2. Continues original logic
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      const generatedCode = pythonGenerator.workspaceToCode(ws);
      this.textCode.set(tabId.toString(), generatedCode);

      // Log the generated code to the console
      //this.logGeneratedCode(tabId, tab.name, generatedCode);
    }

    this.updateSrvList().subscribe();
    this.updateMsgList().subscribe();
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

  /**
   * Logs the code generated from Blockly blocks to the console
   * @param tabId ID of the tab
   * @param tabName Name of the tab
   * @param code Generated code from the blocks
   */
  logGeneratedCode(tabId: number, tabName: string, code: string) {
    const logMessage = ``;
    const tabIdStr = tabId.toString();
    // Add the log to the console output
    this.consolesOutput.set(tabIdStr,
      (this.consolesOutput.get(tabIdStr) || '') + logMessage);

    // Update the displayed console if this is the selected tab
    if (this.selectedTabId === tabId) {
      this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabIdStr) || '';
      if (this.autoScrollEnabled) {
        setTimeout(() => this.scrollToBottom(), 100);
      }
    }
  }

  stopTab(tabId: number) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      // --- Immediate UI Update ---
      tab.isPlaying = false;
      this.changeDetectorRef.detectChanges(); // Ensure UI reflects change quickly
      // --- End Immediate UI Update ---

      const session_id = this.consolesSessions.get(tabId.toString());
      const codeService = this.consolesServices.get(tabId.toString());

      console.log('Stop requested for tab:', tabId, 'Session ID:', session_id);

      if (session_id && codeService) {
        // Request backend termination
        codeService.killExecution(session_id);
        // Note: killExecution doesn't instantly confirm termination.
        // The WebSocket 'terminated' message is the confirmation.
      } else {
         console.warn(`No active session or service found for tab ${tabId} to kill.`);
      }

      // Close the frontend WebSocket connection attempt immediately
      // This prevents receiving further messages after stop is requested
      // The handleTermination/handleError will do the final cleanup upon receiving status msg
      codeService?.closeConnection(); // Close WS from frontend side
      this.websockets.get(tabId.toString())?.unsubscribe(); // Unsubscribe immediately

      // We *don't* immediately remove from websockets/consolesSessions here.
      // handleTermination/handleError triggered by the WebSocket status message
      // will perform the final cleanup. This handles cases where the kill
      // command might fail or take time.
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
          if (category.contents && Array.isArray(category.contents)) {
            const filteredSubcontents = category.contents.map((item: any) => {
              if (item.contents && Array.isArray(item.contents)) {
                const filteredBlocks = item.contents.filter((block: any) => {
                  if (block.kind === 'block' && block.type) {
                    return block.type.toLowerCase().includes(query);
                  } else if (block.kind === 'label' && block.text) {
                    return block.text.toLowerCase().includes(query);
                  }
                  return false;
                });
                return filteredBlocks.length > 0 ? { ...item, contents: filteredBlocks } : null;
              }
              return null;
            }).filter((subcat: any) => subcat !== null);

            return filteredSubcontents.length > 0 ? { ...category, contents: filteredSubcontents } : null;
          }
          return null;
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

    //init testing
    performance.clearMarks();
    performance.clearMeasures();
    performance.mark('all_start');

    this.allRunning = true;
    this.allReady = 0;
    this.seenMsgs = new WeakSet();

    /* cuenta cuántos tabs realmente se lanzarán */
    this.allTotal = this.tabs.filter(tab => this.workspaces[tab.id]).length;

    //end testing

    for (const tab of this.tabs) {
      const tabId = tab.id;
      // Only play the tab if it's not already playing
      if (!tab.isPlaying && this.workspaces[tabId]) {
          console.log(`PlayAll: Attempting to play tab ${tabId} (${tab.name})`);
      this.playTab(tabId, true);
      } else {
          console.log(`PlayAll: Skipping already running or non-existent workspace tab ${tabId} (${tab.name})`);
      }
    }
  }

  stopAllTabs() {
    for (const tab of this.tabs) {
      const tabId = tab.id;
      if (!this.workspaces[tabId]) continue;
      this.stopTab(tabId);
    }
    this.resetTurtleContainer(this.currentMap);
    this.seenMsgs = new WeakSet();
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
    resetNodePerf();
    performance.mark('ps_start');
    console.log(code_to_send);
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
      performance.mark('clickClient');
      fileName = sanitizePythonFilename(this.tabs.find(tab => tab.id === tabId)?.name || 'Cliente');
      code = replaceSelfWithNodeInMain(create_client(linesBeforeComment(code), fileName, linesAfter(code), serverType));
    }
    const codeService = this.consolesServices.get(tabId.toString());
    if (!codeService) {
      console.error('Service not found for the tab', tabId);
      return;
    } else {
      const tab = this.tabs.find(t => t.id === tabId);
      if (!tab) {
          console.error(`Tab with ID ${tabId} not found.`);
          return; // Exit if tab not found
      }

      // --- Flujo Condicional: Guardar Interfaz vs. Ejecutar Nodo ---
      if (type === "srv" || type === "msg") {
          // --- Guardado de Interfaz (HTTP Síncrono Backend) ---
          console.log(`Saving interface: Type=${type}, Name=${fileName}`);
          tab.isPlaying = true; // Indicate processing started (use isPlaying for now)
          this.changeDetectorRef.detectChanges();

          codeService.uploadCode(fileName, code, type).subscribe({
              next: (response) => {
                  console.log(`Interface ${fileName} saved successfully:`, response);
                  tab.isPlaying = false; // Processing finished
                  const confirmationMessage = `Interface ${fileName} saved successfully.\n`;
              this.consolesOutput.set(tabId.toString(),
                    (this.consolesOutput.get(tabId.toString()) ?? '') + confirmationMessage);
              if (this.selectedTabId === tabId) {
                this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) ?? '';
                  }

                  // Update the relevant interface list
                  if (type === "srv") {
                      this.updateSrvList().subscribe(() => {
                          console.log('SRV list updated after save.');
                          // Optionally refresh toolbox immediately if needed
                          // this.updateSrvVariablesCategory();
                      });
                  } else { // type === "msg"
                      this.updateMsgList().subscribe(() => {
                          console.log('MSG list updated after save.');
                          // Optionally refresh toolbox immediately if needed
                          // this.updateMsgVariablesCategory();
                      });
                  }
                  this.changeDetectorRef.detectChanges();
              },
              error: (error) => {
                  console.error(`Error saving interface ${fileName}:`, error);
                  tab.isPlaying = false; // Processing finished (with error)
                  const detail = error?.error?.detail || error.message || 'Unknown error';
                  this.errorsService.showErrors(`Error saving ${fileName}: ${detail}`);
                  this.changeDetectorRef.detectChanges();
              }
              // No 'complete' needed typically for single HTTP POST
          });

      } else {
          // --- Ejecución de Nodo (WebSocket Asíncrono) ---
          console.log(`Executing node: Type=${type}, Name=${fileName}`);
          // Make variables global in each "def"
          code = sanitizeGlobalVariables(code);
          console.log(code);

          // Ensure WebSocket subscription map exists
          if (!this.websockets) {
              this.websockets = new Map<string, Subscription>();
          }

          // Unsubscribe from previous session if any
          if (this.websockets.has(tabId.toString())) {
              this.websockets.get(tabId.toString())?.unsubscribe();
              this.websockets.delete(tabId.toString());
          }

          const executionSubscription = codeService.uploadCode(fileName, code, type)
            .pipe(
              switchMap(() => {
                // This part is skipped for srv/msg types by the outer if/else
                // No need to check type === "srv" || type === "msg" here anymore
            return codeService.executeCode(fileName);
          }),
          switchMap((response) => {
                if (!response) return of(null); // Handle potential null response from executeCode
                console.log('Backend execution response:', response);
            const sessionId = response.session_id;
                // Remove the specific check and error handling for missing session_id
                // if (!sessionId) {
                //     console.error('No session_id received from executeCode');
                //     // Handle error appropriately - maybe show error, set isPlaying false
                //     tab.isPlaying = false;
                //     this.errorsService.showErrors(`Failed to start execution for ${fileName}: No session ID received.`);
                //     this.changeDetectorRef.detectChanges();
                //     return of(null); // Prevent further steps
                // }
                // Proceed assuming sessionId exists (or let connectToWebSocket handle potential issues)
            this.consolesSessions.set(tabId.toString(), sessionId);
            console.log('Session ID:', sessionId);

                // Connect to WebSocket
                const wsConnection$ = codeService.connectToWebSocket(sessionId);

                // Set isPlaying = true only AFTER successful WebSocket connection attempt
                if (tab) { // Check tab again
                    tab.isPlaying = true;
                    this.changeDetectorRef.detectChanges(); // Update UI
                }
                return wsConnection$;
          })
        )
        .subscribe({
          next: (response) => {
                 if (!response) return; // Skip if null (e.g., from error handling above)
                // ... (Existing WebSocket message handling: output vs status) ...
                 // Differentiate between log output and status messages
                 if (response.output !== undefined) {
                   // Log message
            console.log('Websocket message:', response.output);
                   // Append output - RE-ADD the explicit '\\n' here if backend doesn't send it
              this.consolesOutput.set(tabId.toString(), (this.consolesOutput.get(tabId.toString()) ?? '') + response.output + '\n');
              if (this.selectedTabId === tabId) {
                this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString())!;
              }
              if (this.autoScrollEnabled) {
                setTimeout(() => this.scrollToBottom(), 100);
              }
                 } else if (response.status) {
                   // Status message
                   const sessionId = this.consolesSessions.get(tabId.toString());
                   console.log(`Received status: ${response.status} for session: ${sessionId}`);
                   if (response.status === 'terminated') {
                     this.handleTermination(tabId, sessionId ?? '', 'Execution Finished.'); // Call handler
                   } else if (response.status === 'error') {
                     this.handleError(tabId, sessionId ?? '', response.detail || 'Unknown execution error.'); // Call handler
                   }
                   // Backend should close the connection after sending status,
                   // but we can also unsubscribe here as a safety measure.
                   this.websockets.get(tabId.toString())?.unsubscribe();
                   this.websockets.delete(tabId.toString());
                 } else {
                   // Unknown message format
                   console.warn('Received unknown message format via WebSocket:', response);
                 }
              },
              error: (error) => { // Handle WebSocket errors
                console.error('WebSocket Error:', error);
                const sessionId = this.consolesSessions.get(tabId.toString());
                this.handleError(tabId, sessionId ?? '', `WebSocket connection error: ${error.message || 'Unknown error'}`);
              },
              complete: () => { // Handle WebSocket closure by backend
                console.log('WebSocket connection closed by backend.');
                const sessionId = this.consolesSessions.get(tabId.toString());
                if (this.consolesSessions.has(tabId.toString())) { // Check if not already handled
                   console.warn(`WebSocket for session ${sessionId} closed unexpectedly.`);
                   // Don't add message here as per previous request
                   this.handleTermination(tabId, sessionId ?? '', 'Execution finished (connection closed).');
                }
              }
            });
            // Store the subscription to manage it
            this.websockets.set(tabId.toString(), executionSubscription);
      }
    }
  }

  scrollToBottom() {
    const consoleContainer = document.querySelector('.console-output-container');
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
        this.matrix = parseMatrix(content);
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
      name: 'Custom srv types created',
      colour : blockColors.Services,
      contents: srvList.map((service: SrvInfo) => {
        const requestBlocks = service.variables?.request?.map((variable: any) =>
          this.createSrvVariableBlock(variable, "request")
        ) || [];
        const responseBlocks = service.variables?.response?.map((variable: any) =>
          this.createSrvVariableBlock(variable, "response")
        ) || [];
        return {
          kind: 'category',
          type: 'category',
          colour: blockColors.Services,
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
            })
          ]
        };
      })
    };

    updateDeepCategoryInToolbox(
      toolboxObj,
      ['ROS 2 Blocks', 'ROS 2 msg and srv types', 'ROS 2 srv types'],
      'Custom srv types created',
      srvVariablesCategory
    );

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

    const typeToBlocksMap: Record<string, any[]> = {};
  
    let categoryName = 'ROS 2 common msg types';
            
    msgList.forEach((message: MsgInfo) => {
      const baseType = message.name.split('.').pop() || message.name; 
      message.fields?.forEach((field: any) => {
        if (!typeToBlocksMap[baseType]) {
          typeToBlocksMap[baseType] = [];
        }
        const block = this.createMsgVariableBlock(field);
        block.data = message.name;
        typeToBlocksMap[baseType].push(block);
      });
    });


    const flatContents: any[] = [];
    Object.entries(typeToBlocksMap).forEach(([type, blocks]) => {
      flatContents.push({ kind: 'label', text: `${type} message fields:` });
      flatContents.push(...blocks);
    });
  
    const defaultMsgTypesCategory = {
      kind: 'category',
      name: categoryName,
      colour: blockColors.Messages,
      contents: flatContents
    };
    updateDeepCategoryInToolbox(
      toolboxObj,
      ['ROS 2 Blocks', 'ROS 2 msg and srv types', 'ROS 2 msg types'],
      categoryName,
      defaultMsgTypesCategory
    );
  
    // Actualiza el toolbox del workspace activo
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.workspaces[this.selectedTabId].updateToolbox({
        kind: 'categoryToolbox',
        contents: toolboxObj.contents
      });
    }
  }

  updateCustomMsgVariablesCategory(): void {
    const toolboxObj = toolbox.contents && toolbox.contents.length > 0
      ? { ...toolbox }
      : { kind: 'categoryToolbox', contents: [] };
  
    const typeToBlocksMap: Record<string, any[]> = {};
  
    let categoryName = 'Custom created msg types';
    const customOnlyList = customMsgList.filter(
      customMsg => !msgList.some(commonMsg => commonMsg.name === customMsg.name)
    );
    
    customOnlyList.forEach((message: MsgInfo) => {
      const baseType = message.name; 
      message.fields?.forEach((field: any) => {
        if (!typeToBlocksMap[baseType]) {
          typeToBlocksMap[baseType] = [];
        }
        const block = this.createMsgVariableBlock(field);
        block.data = message.name;
        typeToBlocksMap[baseType].push(block);
      });
    });


    const flatContents: any[] = [];
    Object.entries(typeToBlocksMap).forEach(([type, blocks]) => {
      flatContents.push({ kind: 'label', text: `${type} message fields:` });
      flatContents.push(...blocks);
    });
  
    const customMsgTypeCategory = {
      kind: 'category',
      name: categoryName,
      colour: blockColors.Messages,
      contents: flatContents
    };
    updateDeepCategoryInToolbox(
      toolboxObj,
      ['ROS 2 Blocks', 'ROS 2 msg and srv types', 'ROS 2 msg types'],
      categoryName,
      customMsgTypeCategory
    );
  
    // Actualiza el toolbox del workspace activo
    if (this.selectedTabId && this.workspaces[this.selectedTabId]) {
      this.workspaces[this.selectedTabId].updateToolbox({
        kind: 'categoryToolbox',
        contents: toolboxObj.contents
      });
    }
  }
  

  updateSrvList(): Observable<any> {
    return this.codeService.checkSrvFiles().pipe(
      tap({ // Usa el formato de objeto observador aquí
        next: (response) => { // Lógica 'next'
          if (response.exists) {
            srvList.length = 0;
            response.files.forEach((file: any) => {
              srvList.push(file);
            });
          } else {
            srvList.length = 0;
          }
          //console.log("srvList updated:", srvList);
          this.updateSrvVariablesCategory();
        },
        error: (error) => { // Lógica 'error'
          console.error("Error getting list of srv files:", error);
          srvList.length = 0;
          this.updateSrvVariablesCategory(); // Asegúrate de actualizar incluso en error
        }
        // complete: () => console.log('srvList update complete') // Opcional
      })
    );
  }

  updateMsgList(): Observable<any> { // <-- A esta línea
    // return this.codeService.checkMsgFiles().subscribe(response => { // <-- Cambiar esta línea
    return this.codeService.checkMsgFiles().pipe( // <-- A esta línea
      tap({ // <-- Usar la sintaxis de objeto para tap
        next: (response) => {
          if (response.exists) {
            response.files.forEach((file: any) => {
              const alreadyExists = customMsgList.some(msg => msg.name === file.name);
              if (!alreadyExists) {
                customMsgList.push(file);
              }
            });
          }
          //console.log("msgList updated:", msgList);
          this.updateMsgVariablesCategory();
          this.updateCustomMsgVariablesCategory(); 
        },
        error: (error) => {
          console.error("Error getting list of msg files:", error);
          this.updateMsgVariablesCategory(); 
          this.updateCustomMsgVariablesCategory(); 
        }
      })
      // }, error => { ... }); // <-- Eliminar el subscribe anterior
    ); // <-- Añadir paréntesis de cierre para pipe
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
      {
        types: ['ros_create_server'], alertMsg: 'You have just removed a server block, therefore the session will end.', callback: () => {
          const tabName = this.tabs.find(t => t.id === tabId)?.name || '';
          this.stopTab(tabId);
          this.consolesSessions.delete(tabId.toString());
          this.consolesServices.get(tabId.toString())?.deleteFile(tabName);
        }, needConfirmation: false, requirePlayedCheck: true
      },
      {
        types: ['ros2_message_block'],
        alertMsg: 'Are you sure you want to delete this message? This action will cascade all blocks associated with its variables.',
        callback: (xml?: string) => {
          const parser = new DOMParser();
          const xmlDoc = parser.parseFromString(xml || '', "text/xml");
          const messageName = xmlDoc.querySelector('field[name="MESSAGE_NAME"]')?.textContent || '';
          console.log('Initiating asynchronous message deletion for:', messageName);

          if (!messageName) {
            console.error("Could not extract message name from deleted block XML.");
            return;
          }
          const normalizedMessageName = messageName.replace(/\\.msg$/, ""); // Normalize here

          const currentWorkspace = this.workspaces[tabId];
          if (!currentWorkspace) {
            console.error(`Workspace with ID ${tabId} not found for immediate cleanup.`);
            // Continue with backend deletion and other tabs if possible
          }

          // Paso 1: Limpiar la pestaña ACTUAL inmediatamente
          console.log(`Cleaning current tab (${tabId}) synchronously for message ${normalizedMessageName}...`);
          try {
            const messageInfo = customMsgList.find(m =>
              (m.name && m.name.replace(/\\.msg$/, '') === normalizedMessageName) // Use normalized name
            );
            if (messageInfo && messageInfo.fields && currentWorkspace) {
              // Call the specific cascade function for messages
              this.cascadeDeleteMessageVariablesInWorkspace(currentWorkspace, normalizedMessageName, messageInfo.fields);
            } else if (currentWorkspace) {
              // Call the general message deletion if specific info not found (e.g., list outdated)
              this.globalMessageBlockDeletedInWorkspace(currentWorkspace, normalizedMessageName);
            }
          } catch (e) {
            console.error(`Error cleaning current workspace ${tabId} for message ${normalizedMessageName}:`, e);
          }

          // Paso 2: Planificar la limpieza de OTRAS pestañas de forma asíncrona
          Object.keys(this.workspaces).forEach(wsTabKey => {
            const otherTabId = +wsTabKey;
            // Saltar la pestaña actual que ya se limpió
            if (otherTabId === tabId) {
              return;
            }

            const otherWorkspace = this.workspaces[otherTabId];
            if (otherWorkspace) { // Check if workspace exists
              console.log(`Scheduling cleanup for tab ${otherTabId} for message ${normalizedMessageName} in 10ms`);
              setTimeout(() => {
                console.log(`Executing asynchronous cleanup for tab ${otherTabId} for message ${normalizedMessageName}`);
                try {
                   const messageInfo = customMsgList.find(m =>
                     (m.name && m.name.replace(/\\.msg$/, '') === normalizedMessageName)
                   );
                  // Re-check workspace existence inside timeout
                  if (this.workspaces[otherTabId]) { // Re-check existence
                    if (messageInfo && messageInfo.fields) {
                      this.cascadeDeleteMessageVariablesInWorkspace(this.workspaces[otherTabId], normalizedMessageName, messageInfo.fields);
                    } else {
                      this.globalMessageBlockDeletedInWorkspace(this.workspaces[otherTabId], normalizedMessageName);
                    }
                  } else {
                    console.log(`Workspace ${otherTabId} was disposed before scheduled message cleanup could run.`);
                  }
                } catch (e) {
                  console.error(`Error during asynchronous cleanup of workspace ${otherTabId} for message ${normalizedMessageName}:`, e);
                }
              }, 10); // Small delay
            } else {
              console.log(`Skipping scheduling cleanup for already disposed or non-existent workspace ${otherTabId}`);
            }
          });

          // Paso 3: Detener la ejecución de la pestaña actual (si estaba en ejecución)
          console.log(`Stopping tab ${tabId} if it was running (message deletion).`);
          this.stopTab(tabId); // Stop regardless of message/service type

          // Paso 4: Eliminar el mensaje en el backend y actualizar la lista
          console.log(`Calling backend to delete message file: ${normalizedMessageName}`);
          // Use 'msg' for type
          this.codeService.deleteInterfaceFile('msg', normalizedMessageName)
            .subscribe({
              next: (response) => {
                console.log("Backend message deletion successful:", response);

                // Actualizar la lista local de mensajes
                const index = customMsgList.findIndex(m =>
                  (m.name && m.name.replace(/\\.msg$/, '') === normalizedMessageName)
                );

                if (index !== -1) {
                  customMsgList.splice(index, 1);
                  console.log('msgList updated after deleting message:', JSON.stringify(customMsgList));
                  // Forzar actualización del toolbox
                  setTimeout(() => {
                    this.updateMsgVariablesCategory(); // Update message category
                    // Refresh toolbox in the current tab if it exists
                    if (this.workspaces[tabId]) {
                      try {
                        this.workspaces[tabId].refreshToolboxSelection();
                        console.log(`Toolbox refreshed for tab ${tabId} after message deletion`);
                      } catch (e) {
                        console.error(`Error refreshing toolbox for tab ${tabId} after message deletion: `, e);
                      }
                    }
                  }, 100); // Delay
                } else {
                  console.warn('Deleted message not found in local msgList:', normalizedMessageName, ' Triggering full update.');
                  this.updateMsgList().subscribe(); // Fetch full list if local removal failed
                }
              },
              error: (error) => {
                console.error("Error deleting message interface file from backend:", error);
                this.updateMsgList().subscribe(); // Attempt to update list even on error
              }
            });
        },
        needConfirmation: true,
        requirePlayedCheck: true
      },
      {
        types: ['ros2_service_block'],
        alertMsg: 'Are you sure you want to delete this service? This action will cascade all blocks associated with its variables.',
        callback: (xml?: string) => {
          const parser = new DOMParser();
          const xmlDoc = parser.parseFromString(xml || '', "text/xml");
          const serviceName = xmlDoc.querySelector('field[name="SERVICE_NAME"]')?.textContent || '';
          console.log('Initiating asynchronous service deletion for:', serviceName);

          if (!serviceName) {
            console.error("Could not extract service name from deleted block XML.");
            return;
          }

          const currentWorkspace = this.workspaces[tabId];
          if (!currentWorkspace) {
            console.error(`Workspace with ID ${tabId} not found for immediate cleanup.`);
            // Continue with backend deletion and other tabs if possible
          }

          // Paso 1: Limpiar la pestaña ACTUAL inmediatamente
          console.log(`Cleaning current tab (${tabId}) synchronously...`);
          try {
            const serviceInfo = srvList.find(s =>
              (s.name && s.name.replace('.srv', '') === serviceName)
            );
            if (serviceInfo && serviceInfo.variables && currentWorkspace) {
              this.cascadeDeleteServiceVariablesInWorkspace(currentWorkspace, serviceName, serviceInfo.variables);
            } else if (currentWorkspace) {
              this.globalServiceBlockDeletedInWorkspace(currentWorkspace, serviceName);
            }
          } catch (e) {
            console.error(`Error cleaning current workspace ${tabId}:`, e);
          }

          // Paso 2: Planificar la limpieza de OTRAS pestañas de forma asíncrona
          Object.keys(this.workspaces).forEach(wsTabKey => {
            const otherTabId = +wsTabKey;
            // Saltar la pestaña actual que ya se limpió
            if (otherTabId === tabId) {
              return;
            }

            const otherWorkspace = this.workspaces[otherTabId];
            if (otherWorkspace) { // Check if workspace exists (it wouldn't if disposed and removed)
              console.log(`Scheduling cleanup for tab ${otherTabId} in 10ms`);
              setTimeout(() => {
                console.log(`Executing asynchronous cleanup for tab ${otherTabId} for service ${serviceName}`);
                try {
                  const serviceInfo = srvList.find(s =>
                    (s.name && s.name.replace('.srv', '') === serviceName)
                  );
                  // Re-check workspace existence inside timeout, as it might be disposed
                  if (this.workspaces[otherTabId]) { // Re-check existence
                    if (serviceInfo && serviceInfo.variables) {
                      this.cascadeDeleteServiceVariablesInWorkspace(this.workspaces[otherTabId], serviceName, serviceInfo.variables);
                    } else {
                      this.globalServiceBlockDeletedInWorkspace(this.workspaces[otherTabId], serviceName);
                    }
                  } else {
                    console.log(`Workspace ${otherTabId} was disposed before scheduled cleanup could run.`);
                  }
                } catch (e) {
                  console.error(`Error during asynchronous cleanup of workspace ${otherTabId}:`, e);
                }
              }, 10); // Pequeño retraso para permitir que la UI respire
            } else {
              console.log(`Skipping scheduling cleanup for already disposed or non-existent workspace ${otherTabId}`);
            }
          });

          // Paso 3: Detener la ejecución de la pestaña actual (si estaba en ejecución)
          // Esto se puede hacer aquí o antes, dependiendo de si la limpieza necesita el estado activo
          console.log(`Stopping tab ${tabId} if it was running.`);
          this.stopTab(tabId);

          // Paso 4: Eliminar el servicio en el backend y actualizar la lista
          console.log(`Calling backend to delete service file: ${serviceName}`);
          this.codeService.deleteInterfaceFile('srv', serviceName)
            .subscribe({
              next: (response) => {
                console.log("Backend service deletion successful:", response);

                // Actualizar la lista local de servicios (ya existe lógica similar)
                const index = srvList.findIndex(s =>
                  (s.name && s.name.replace('.srv', '') === serviceName)
                );

                if (index !== -1) {
                  srvList.splice(index, 1);
                  console.log('srvList updated after deleting service:', JSON.stringify(srvList));
                  // Forzar actualización del toolbox en la pestaña actual (y otras si es necesario)
                  setTimeout(() => {
                    this.updateSrvVariablesCategory();
                    // Refresh toolbox in the current tab if it exists
                    if (this.workspaces[tabId]) { // Check if workspace still exists
                      try {
                        this.workspaces[tabId].refreshToolboxSelection();
                        console.log(`Toolbox refreshed for tab ${tabId}`);
                      } catch (e) {
                        console.error(`Error refreshing toolbox for tab ${tabId}: `, e);
                      }
                    }
                    // Consider refreshing other tabs' toolboxes as well if needed
                  }, 100); // Delay to ensure list update propagates
                } else {
                  console.warn('Deleted service not found in local srvList:', serviceName, ' Triggering full update.');
                  this.updateSrvList().subscribe(); // Fetch full list if local removal failed
                }
              },
              error: (error) => {
                console.error("Error deleting service interface file from backend:", error);
                this.updateSrvList().subscribe(); // Attempt to update list even on error
              }
            });
        },
        needConfirmation: true,
        requirePlayedCheck: true // Mantener o ajustar según sea necesario
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
          // 1. Custom srv types created (request/response)
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
          console.error(`Error processing block ${block.id} of type ${block.type}:`, e);
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
    console.log(`Cascading Variable Elimination for Service ${normalizedServiceName}`);

    // Browse all workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      const blocksToDelete: Blockly.Block[] = [];
      const processedBlocks = new Set<string>(); // Para evitar procesar el mismo bloque varias veces

      // Función recursiva para identificar bloques a eliminar
      const findBlocksToDelete = (block: any, serviceToDelete: string) => {
        if (!block || processedBlocks.has(block.id)) return;
        processedBlocks.add(block.id);

        let shouldDelete = false;
        let reason = "";

        try {
          // MÉTODO 1: Examinar propiedades directas del bloque

          // Revisar si el bloque tiene un atributo 'data' con el nombre del servicio
          if (block.data) {
            const blockService = block.data.toString();
            const blockServiceNormalized = blockService.replace(/\.srv$/, "");
            if (blockServiceNormalized === serviceToDelete) {
              shouldDelete = true;
              reason = `Block has data attribute matching service ${serviceToDelete}`;
            }
          }

          // Revisar campos específicos según el tipo de bloque
          if (!shouldDelete) {
            const fieldsToCheck = [];

            // Verificar campos específicos por tipo de bloque
            if (block.type === 'ros_create_client' || block.type === 'ros_create_server') {
              fieldsToCheck.push('SERVICE_NAME', 'SERVER_TYPE', 'CLIENT_TYPE');
            } else if (block.type === 'ros_send_request') {
              fieldsToCheck.push('SERVICE_TYPE');
            }

            // Verificar todos los posibles nombres de campo
            for (const fieldName of fieldsToCheck) {
              try {
                const fieldValue = block.getFieldValue(fieldName);
                if (fieldValue) {
                  const normalizedValue = fieldValue.toString().replace(/\.srv$/, "");
                  if (normalizedValue === serviceToDelete) {
                    shouldDelete = true;
                    reason = `Block has field ${fieldName} matching service ${serviceToDelete}`;
                    break;
                  }
                }
              } catch (e) {
                // Ignorar errores al intentar obtener valores de campos inexistentes
              }
            }
          }

          // MÉTODO 2: Examinar el contexto (cadena de padres)
          if (!shouldDelete) {
            let parent = block.getParent();
            let depth = 0;
            const maxDepth = 5; // Evitar bucles infinitos o análisis excesivo

            while (parent && depth < maxDepth) {
              // ¿El padre utiliza directamente este servicio?
              const serviceFields = ['SERVICE_NAME', 'SERVER_TYPE', 'CLIENT_TYPE'];
              for (const fieldName of serviceFields) {
                try {
                  const parentServiceField = parent.getFieldValue(fieldName);
                  if (parentServiceField) {
                    const normalizedParentService = parentServiceField.toString().replace(/\.srv$/, "");
                    if (normalizedParentService === serviceToDelete) {
                      shouldDelete = true;
                      reason = `Block is child of a ${parent.type} using service ${serviceToDelete}`;
                      break;
                    }
                  }
                } catch (e) {
                  // Ignorar errores al intentar obtener valores de campos inexistentes
                }
              }

              // ¿El padre tiene datos que coincidan con este servicio?
              if (!shouldDelete && parent.data) {
                const parentData = parent.data.toString();
                const normalizedParentData = parentData.replace(/\.srv$/, "");
                if (normalizedParentData === serviceToDelete) {
                  shouldDelete = true;
                  reason = `Block is child of a block with data attribute matching service ${serviceToDelete}`;
                }
              }

              if (shouldDelete) break;
              parent = parent.getParent();
              depth++;
            }
          }

          // MÉTODO 3: Examinar las mutaciones del bloque
          if (!shouldDelete && block.mutationToDom) {
            try {
              const mutation = block.mutationToDom();
              if (mutation) {
                const attrs = Array.from(mutation.attributes) as Attr[];
                for (const attr of attrs) {
                  const attrValue = attr.value.toString();
                  // Buscar coincidencias en atributos de mutación
                  if (attrValue.includes(serviceToDelete) ||
                    attrValue.includes(serviceToDelete + '.srv')) {
                    shouldDelete = true;
                    reason = `Block has mutation attribute containing service ${serviceToDelete}`;
                    break;
                  }

                  // Intentar parsear JSON en atributos
                  try {
                    const jsonValue = JSON.parse(attrValue);
                    if (typeof jsonValue === 'string' &&
                      (jsonValue === serviceToDelete || jsonValue === serviceToDelete + '.srv')) {
                      shouldDelete = true;
                      reason = `Block has JSON mutation data matching service ${serviceToDelete}`;
                      break;
                    }
                  } catch (e) {
                    // No es JSON válido, ignorar
                  }
                }
              }
            } catch (e) {
              // Error al obtener mutación, ignorar
            }
          }

          // Si el bloque debe ser eliminado, lo añadimos a la lista
          if (shouldDelete && !blocksToDelete.includes(block)) {
            console.log(`Found block ${block.id} of type ${block.type} related to service ${serviceToDelete}: ${reason}`);
            blocksToDelete.push(block);
          }

          // Continuamos la búsqueda recursiva en todos los bloques conectados
          // 1. Examinar todos los inputs
          if (block.inputList) {
            block.inputList.forEach((input: any) => {
              if (input.connection && input.connection.targetBlock()) {
                findBlocksToDelete(input.connection.targetBlock(), serviceToDelete);
              }
            });
          }

          // 2. Examinar el siguiente bloque
          if (block.nextConnection && block.nextConnection.targetBlock()) {
            findBlocksToDelete(block.nextConnection.targetBlock(), serviceToDelete);
          }

          // 3. Examinar los bloques conectados como salida (output)
          if (block.outputConnection && block.outputConnection.targetBlock()) {
            findBlocksToDelete(block.outputConnection.targetBlock(), serviceToDelete);
          }

        } catch (e) {
          console.error(`Error analyzing block ${block.id} of type ${block.type}:`, e);
        }
      };

      // Comenzamos la búsqueda desde todos los bloques de nivel superior
      workspace.getTopBlocks().forEach(block => {
        findBlocksToDelete(block, normalizedServiceName);
      });

      // Eliminamos los bloques identificados en orden inverso
      blocksToDelete.reverse().forEach(block => {
        try {
          console.log(`Deleting block ${block.id} of type ${block.type} on tab ${tabKey}`);
          block.dispose(true); // true para eliminar bloques conectados
        } catch (e) {
          console.error("Error deleting block:", e);
        }
      });
    });
  }

  // Add this new feature to handle cascading deletion of blocks associated with a service
  // Modified to operate on a specific workspace to prevent UI freezing
  globalServiceBlockDeletedInWorkspace(workspace: Blockly.WorkspaceSvg, serviceName: string): void {
    if (!workspace) {
      console.warn(`Attempted to clean a null or undefined workspace for service ${serviceName}.`);
      return;
    }
    console.log(`Searching for blocks dependent on service ${serviceName} in workspace ${workspace.id}`);

    // Service name without extension for more flexible comparisons
    const normalizedServiceName = serviceName.replace(/\.srv$/, "");

    // Removed the loop: Object.keys(this.workspaces).forEach((tabKey) => { ... });
    // Now operates directly on the provided workspace

    const allBlocks = workspace.getAllBlocks(); // Use the passed workspace

    console.log(`Reviewing workspace ${workspace.id} (${allBlocks.length} blocks) for service ${normalizedServiceName}`);

    // Identify all blocks that need to be removed within this specific workspace
    const blocksToRemove: Blockly.Block[] = []; // More specific type

    allBlocks.forEach((block: Blockly.Block) => { // More specific type
      try {
        // 1. Custom srv types created (request/response)
        if (block.type === 'srv_variable') {
          // Check if the block belongs to the service through its fields or data attribute
          const variableName = block.getFieldValue('VAR_NAME');
          const variableSection = block.getFieldValue('VAR_SECTION');
          const blockDataService = block.data ? String(block.data).replace(/\.srv$/, "") : null;

          if (blockDataService === normalizedServiceName) {
            console.log(`  Found srv_variable block ${block.id} via data attribute`);
            blocksToRemove.push(block);
          } else {
            // Fallback check using srvList (less reliable if list is stale)
            const serviceInfo = srvList.find(s =>
              (s.name && s.name.replace('.srv', '') === normalizedServiceName)
            );

            if (serviceInfo) {
              const isInService = (variableSection === 'request' && serviceInfo.variables?.request?.some(v => v.name === variableName)) ||
                (variableSection === 'response' && serviceInfo.variables?.response?.some(v => v.name === variableName));

              if (isInService) {
                console.log(`  Variable block found ${variableName} (${variableSection}) for service ${normalizedServiceName} via srvList`);
                blocksToRemove.push(block);
              }
            }
          }
        }

        // 2. Response assignment blocks
        else if (block.type === 'srv_response_set_field') {
          // Check context (parent) or potentially connected blocks if parent isn't direct client/server
          let isRelatedToService = false;
          let parent = block.getParent();
          if (parent && (parent.type === 'ros_create_client' || parent.type === 'ros_create_server')) {
            const parentServiceTypeField = parent.type === 'ros_create_client' ? 'CLIENT_TYPE' : 'SERVER_TYPE';
            const parentServiceName = parent.getFieldValue(parentServiceTypeField);
            if (parentServiceName && parentServiceName.replace(/\.srv$/, "") === normalizedServiceName) {
              isRelatedToService = true;
            }
          }
          // Consider adding more checks if needed, e.g., checking the connected block to RESPONSE_FIELD

          if (isRelatedToService) {
            console.log(`  Found response assignment block ${block.id} related to service ${normalizedServiceName}`);
            blocksToRemove.push(block);
          }
        }

        // 3. Client/Server blocks that use the service
        else if (block.type === 'ros_create_client' || block.type === 'ros_create_server') {
          const serviceTypeField = block.type === 'ros_create_client' ? 'CLIENT_TYPE' : 'SERVER_TYPE';
          const blockServiceName = block.getFieldValue(serviceTypeField);
          if (blockServiceName && blockServiceName.replace(/\.srv$/, "") === normalizedServiceName) {
            console.log(`  Found ${block.type} block ${block.id} using service ${normalizedServiceName}`);
            blocksToRemove.push(block);
          }
        }

        // 4. Send Request blocks (might need parent check)
        else if (block.type === 'ros_send_request') {
          let parent = block.getParent();
          if (parent && parent.type === 'ros_create_client') {
            const clientServiceType = parent.getFieldValue('CLIENT_TYPE');
            if (clientServiceType && clientServiceType.replace(/\.srv$/, "") === normalizedServiceName) {
              console.log(`  Found send_request block ${block.id} inside client for service ${normalizedServiceName}`);
              blocksToRemove.push(block);
            }
          }
        }

      } catch (e) {
        console.error(`Error processing block ${block.id} of type ${block.type}:`, e);
      }
    });

    // Use a Set to ensure uniqueness before disposing
    const uniqueBlocksToRemove = [...new Set(blocksToRemove)];

    // Delete all identified blocks within this workspace, with their dependents
    console.log(`Disposing ${uniqueBlocksToRemove.length} blocks in workspace ${workspace.id}`);
    uniqueBlocksToRemove.forEach(block => {
      if (!block.isDisposed()) { // Check if already disposed
        try {
          console.log(`  Deleting block ${block.id} of type ${block.type}`);
          block.dispose(true); // true to dispose children/connections
        } catch (e) {
          console.error(`  Error deleting block ${block.id}:`, e);
        }
      } else {
        console.log(`  Block ${block.id} was already disposed.`);
      }
    });
    // End of modified function scope
  }

  // Modified to operate on a specific workspace
  cascadeDeleteServiceVariablesInWorkspace(workspace: Blockly.WorkspaceSvg, serviceName: string, serviceVariables: { request?: any[]; response?: any[]; }): void {
    if (!workspace) {
      console.warn(`Attempted to cascade delete in a null or undefined workspace for service ${serviceName}.`);
      return;
    }
    const normalizedServiceName = serviceName.replace(/\.srv$/, "");
    console.log(`Cascading Variable Elimination for Service ${normalizedServiceName} in workspace ${workspace.id}`);

    // Removed the loop: Object.keys(this.workspaces).forEach((tabKey) => { ... });
    // Operates directly on the provided workspace

    const blocksToDelete: Blockly.Block[] = [];
    const processedBlocks = new Set<string>(); // To avoid processing the same block multiple times

    // Recursive function to identify blocks to delete within this workspace
    const findBlocksToDelete = (block: any, serviceToDelete: string) => { // Use Blockly.Block type
      if (!block || processedBlocks.has(block.id)) return; // Check if null or already processed
      processedBlocks.add(block.id);

      let shouldDelete = false;
      let reason = "";

      try {
        // METHOD 1: Examine direct properties of the block
        if (block.data) {
          const blockService = String(block.data).replace(/\.srv$/, ""); // Ensure string conversion
          if (blockService === serviceToDelete) {
            shouldDelete = true;
            reason = `Block ${block.id} has data attribute matching service ${serviceToDelete}`;
          }
        }

        // Check specific fields based on block type
        if (!shouldDelete) {
          const fieldsToCheck: string[] = []; // Specify type
          if (block.type === 'ros_create_client' || block.type === 'ros_create_server') {
            fieldsToCheck.push('SERVICE_NAME', 'SERVER_TYPE', 'CLIENT_TYPE');
          } else if (block.type === 'ros_send_request') {
            // Check parent context for send_request instead of direct field
          } else if (block.type === 'srv_variable') {
            // Handled by data attribute or context usually, but could add field check if needed
          }
          // Add other relevant block types and their fields here

          for (const fieldName of fieldsToCheck) {
            try {
              const fieldValue = block.getFieldValue(fieldName);
              if (fieldValue) {
                const normalizedValue = String(fieldValue).replace(/\.srv$/, ""); // Ensure string conversion
                if (normalizedValue === serviceToDelete) {
                  shouldDelete = true;
                  reason = `Block ${block.id} has field ${fieldName} matching service ${serviceToDelete}`;
                  break;
                }
              }
            } catch (e) {
              // Ignorar errores al intentar obtener valores de campos inexistentes
            }
          }
        }

        // METHOD 2: Examine context (parent chain)
        if (!shouldDelete) {
          let parent = block.getParent();
          let depth = 0;
          const maxDepth = 5;

          while (parent && depth < maxDepth) {
            const serviceFields = ['SERVICE_NAME', 'SERVER_TYPE', 'CLIENT_TYPE'];
            let parentUsesService = false;
            for (const fieldName of serviceFields) {
              try {
                const parentServiceField = parent.getFieldValue(fieldName);
                if (parentServiceField) {
                  const normalizedParentService = String(parentServiceField).replace(/\.srv$/, "");
                  if (normalizedParentService === serviceToDelete) {
                    parentUsesService = true;
                    break;
                  }
                }
              } catch (e) {
                // Ignorar errores al intentar obtener valores de campos inexistentes
              }
            }

            if (!parentUsesService && parent.data) {
              const parentData = String(parent.data).replace(/\.srv$/, ""); // Declare and normalize here
              if (parentData === serviceToDelete) { // Use the declared variable
                parentUsesService = true;
              }
            }

            if (parentUsesService) {
              shouldDelete = true;
              reason = `Block ${block.id} is child of ${parent.type} (${parent.id}) using service ${serviceToDelete}`;
              break;
            }

            parent = parent.getParent();
            depth++;
          }
        }

        // METHOD 3: Examine block mutations (if applicable)
        if (!shouldDelete && typeof (block as any).mutationToDom === 'function') {
          try {
            const mutation = (block as any).mutationToDom();
            if (mutation && mutation.attributes) {
              const attrs = Array.from(mutation.attributes) as Attr[];
              for (const attr of attrs) {
                const attrValue = attr.value.toString();
                if (attrValue.includes(serviceToDelete) || attrValue.includes(serviceToDelete + '.srv')) {
                  shouldDelete = true;
                  reason = `Block ${block.id} has mutation attribute containing service ${serviceToDelete}`;
                  break;
                }

                // Optional: Add JSON parsing check if needed
              }
            }
          } catch (e) {
            // Error al obtener mutación, ignorar
          }
        }

        // If the block should be deleted, add it to the list
        if (shouldDelete && !blocksToDelete.includes(block)) {
          console.log(`  Found block ${block.id} of type ${block.type} related to service ${serviceToDelete}: ${reason}`);
          blocksToDelete.push(block);
        }

        // Continue recursive search in connected blocks
        if (block.inputList) {
          block.inputList.forEach((input: any) => {
            if (input.connection && input.connection.targetBlock()) {
              findBlocksToDelete(input.connection.targetBlock(), serviceToDelete);
            }
          });
        }

        // 2. Examinar el siguiente bloque
        if (block.nextConnection && block.nextConnection.targetBlock()) {
          findBlocksToDelete(block.nextConnection.targetBlock(), serviceToDelete);
        }

        // 3. Examinar los bloques conectados como salida (output)
        if (block.outputConnection && block.outputConnection.targetBlock()) {
          // Be careful with output connections, might not always imply deletion cascade
          // Depending on logic, you might want to check the parent instead or only specific types
          // findBlocksToDelete(block.outputConnection.targetBlock(), serviceToDelete);
        }

      } catch (e) {
        console.error(`Error analyzing block ${block.id} of type ${block.type}:`, e);
      }
    };

    // Start the search from all top-level blocks in this workspace
    workspace.getTopBlocks().forEach(block => {
      findBlocksToDelete(block, normalizedServiceName);
    });

    // Use a Set to ensure uniqueness before disposing
    const uniqueBlocksToDelete = [...new Set(blocksToDelete)];

    // Delete the identified blocks in reverse order
    console.log(`Disposing ${uniqueBlocksToDelete.length} blocks via cascade in workspace ${workspace.id}`);
    uniqueBlocksToDelete.reverse().forEach(block => {
      if (!block.isDisposed()) { // Check if not already disposed
        try {
          console.log(`  Deleting block ${block.id} of type ${block.type}`);
          block.dispose(true); // true to delete connected blocks
        } catch (e) {
          console.error(`  Error deleting block ${block.id}:`, e);
        }
      } else {
        console.log(` Block ${block.id} was already disposed (cascade).`);
      }
    });
    // End of modified function scope
  }

  // Add this new feature to handle cascading deletion of blocks associated with a message
  globalMessageBlockDeleted(messageName: string): void {
    console.log(`Searching for blocks dependent on the deleted message: ${messageName}`);

    // Message name without extension for more flexible comparisons
    const normalizedMessageName = messageName.replace(/\\.msg$/, "");

    // Cycle through all tabs/workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      if (workspace) { // Check if workspace exists
        this.globalMessageBlockDeletedInWorkspace(workspace, normalizedMessageName);
      }
    });
  }

  // Add this new feature to handle cascading deletion of blocks associated with a message
  // Modified to operate on a specific workspace to prevent UI freezing
  globalMessageBlockDeletedInWorkspace(workspace: Blockly.WorkspaceSvg, messageName: string): void {
    if (!workspace) {
      console.warn(`Attempted to clean a null or undefined workspace for message ${messageName}.`);
      return;
    }
    console.log(`Searching for blocks dependent on message ${messageName} in workspace ${workspace.id}`);

    // Message name without extension for more flexible comparisons
    const normalizedMessageName = messageName.replace(/\\.msg$/, ""); // Use regex for extension removal

    const allBlocks = workspace.getAllBlocks(); // Use the passed workspace

    console.log(`Reviewing workspace ${workspace.id} (${allBlocks.length} blocks) for message ${normalizedMessageName}`);

    // Identify all blocks that need to be removed within this specific workspace
    const blocksToRemove: Blockly.Block[] = []; // More specific type

    allBlocks.forEach((block: Blockly.Block) => { // More specific type
      try {
        // 1. Message variables
        if (block.type === 'msg_variable') {
          // Check if the block belongs to the message through its fields or data attribute
          // NOTE: msg_variable currently doesn't store the parent message name directly in data or fields.
          // We might need to check its parent block's context or rely on cascade deletion triggered by the main message block.
          // Let's try checking parent context.
          let parent = block.getParent();
          let related = false;
          while (parent) {
            if ((parent.type === 'ros2_create_publisher' || parent.type === 'ros2_create_subscriber' || parent.type === 'ros2_publish_message')) {
              const parentMsgType = parent.getFieldValue('MSG_TYPE');
              if (parentMsgType && parentMsgType.replace(/\\.msg$/, "") === normalizedMessageName) {
                related = true;
                break;
              }
            } else if (parent.type === 'ros2_message_block') {
              const parentMsgName = parent.getFieldValue('MESSAGE_NAME');
              if (parentMsgName && parentMsgName.replace(/\\.msg$/, "") === normalizedMessageName) {
                related = true;
                break;
              }
            }
            parent = parent.getParent();
          }

          if (related) {
            console.log(`  Found msg_variable block ${block.id} related to message ${normalizedMessageName}`);
            blocksToRemove.push(block);
          }
        }

        // 2. Publisher blocks using the message type
        else if (block.type === 'ros2_create_publisher' || block.type === 'ros2_publish_message') {
          const msgType = block.getFieldValue('MSG_TYPE');
          if (msgType && msgType.replace(/\\.msg$/, "") === normalizedMessageName) {
            console.log(`  Found ${block.type} block ${block.id} using message ${normalizedMessageName}`);
            blocksToRemove.push(block);
          }
        }

        // 3. Subscriber blocks using the message type
        else if (block.type === 'ros2_create_subscriber') {
          const msgType = block.getFieldValue('MSG_TYPE');
          if (msgType && msgType.replace(/\\.msg$/, "") === normalizedMessageName) {
            console.log(`  Found ${block.type} block ${block.id} using message ${normalizedMessageName}`);
            blocksToRemove.push(block);
          }
        }

        // 4. Message definition block itself (should already be handled by the initial delete event, but good for cascade)
        else if (block.type === 'ros2_message_block') {
          const msgName = block.getFieldValue('MESSAGE_NAME');
          if (msgName && msgName.replace(/\\.msg$/, "") === normalizedMessageName) {
            console.log(`  Found message definition block ${block.id} for ${normalizedMessageName}`);
            blocksToRemove.push(block);
          }
        }

      } catch (e) {
        console.error(`Error processing block ${block.id} of type ${block.type} for message ${messageName}:`, e);
      }
    });

    // Use a Set to ensure uniqueness before disposing
    const uniqueBlocksToRemove = [...new Set(blocksToRemove)];

    // Delete all identified blocks within this workspace, with their dependents
    console.log(`Disposing ${uniqueBlocksToRemove.length} blocks related to message ${normalizedMessageName} in workspace ${workspace.id}`);
    uniqueBlocksToRemove.forEach(block => {
      if (!block.isDisposed()) { // Check if already disposed
        try {
          console.log(`  Deleting block ${block.id} of type ${block.type}`);
          block.dispose(true); // true to dispose children/connections
        } catch (e) {
          console.error(`  Error deleting block ${block.id}:`, e);
        }
      } else {
        console.log(`  Block ${block.id} was already disposed.`);
      }
    });
    // End of modified function scope
  }

  cascadeDeleteMessageVariables(messageName: string, messageFields: MsgVariable[]): void {
    const normalizedMessageName = messageName.replace(/\\.msg$/, "");
    console.log(`Cascading Variable Elimination for Message ${normalizedMessageName}`);

    // Browse all workspaces
    Object.keys(this.workspaces).forEach((tabKey) => {
      const workspace = this.workspaces[+tabKey];
      if (workspace) { // Check if workspace exists
        this.cascadeDeleteMessageVariablesInWorkspace(workspace, normalizedMessageName, messageFields);
      }
    });
  }

  // Modified to operate on a specific workspace for message variables
  cascadeDeleteMessageVariablesInWorkspace(workspace: Blockly.WorkspaceSvg, messageName: string, messageFields: MsgVariable[]): void {
    if (!workspace) {
      console.warn(`Attempted to cascade delete message variables in a null or undefined workspace for message ${messageName}.`);
      return;
    }
    const normalizedMessageName = messageName.replace(/\\.msg$/, "");
    console.log(`Cascading Variable Elimination for Message ${normalizedMessageName} in workspace ${workspace.id}`);


    const blocksToDelete: Blockly.Block[] = [];
    const processedBlocks = new Set<string>(); // To avoid processing the same block multiple times

    // Recursive function to identify blocks to delete within this workspace
    const findBlocksToDelete = (block: any, msgToDelete: string) => { // Use Blockly.Block type
      if (!block || processedBlocks.has(block.id)) return; // Check if null or already processed
      processedBlocks.add(block.id);

      let shouldDelete = false;
      let reason = "";

      try {
        // Check specific block types related to messages
        if (block.type === 'msg_variable') {
          // Need to determine if this variable belongs to the deleted message.
          // This requires checking parent context as msg_variable doesn't store parent message name.
          let parent = block.getParent();
          while (parent) {
            let parentMsgName = null;
            if (parent.type === 'ros2_create_publisher' || parent.type === 'ros2_create_subscriber' || parent.type === 'ros2_publish_message') {
              parentMsgName = parent.getFieldValue('MSG_TYPE');
            } else if (parent.type === 'ros2_message_block') {
              parentMsgName = parent.getFieldValue('MESSAGE_NAME');
            }

            if (parentMsgName) {
              const normalizedParentMsgName = parentMsgName.replace(/\\.msg$/, "");
              if (normalizedParentMsgName === msgToDelete) {
                shouldDelete = true;
                reason = `Block ${block.id} (msg_variable) is child of ${parent.type} using message ${msgToDelete}`;
                break;
              }
            }
            parent = parent.getParent();
          }

        } else if (block.type === 'ros2_publish_message' || block.type === 'ros2_create_publisher' || block.type === 'ros2_create_subscriber') {
          const msgType = block.getFieldValue('MSG_TYPE');
          if (msgType) {
            const normalizedMsgType = msgType.replace(/\\.msg$/, "");
            if (normalizedMsgType === msgToDelete) {
              shouldDelete = true;
              reason = `Block ${block.id} (${block.type}) uses message type ${msgToDelete}`;
            }
          }
        }
        // Potentially add checks for blocks that might *contain* msg_variable indirectly

        // If the block should be deleted, add it to the list
        if (shouldDelete && !blocksToDelete.includes(block)) {
          console.log(`  Found block ${block.id} of type ${block.type} related to message ${msgToDelete}: ${reason}`);
          blocksToDelete.push(block);
        }

        // Continue recursive search in connected blocks
        if (block.inputList) {
          block.inputList.forEach((input: any) => {
            if (input.connection && input.connection.targetBlock()) {
              findBlocksToDelete(input.connection.targetBlock(), msgToDelete);
            }
          });
        }

        // 2. Examinar el siguiente bloque
        if (block.nextConnection && block.nextConnection.targetBlock()) {
          findBlocksToDelete(block.nextConnection.targetBlock(), msgToDelete);
        }

      } catch (e) {
        console.error(`Error analyzing block ${block.id} of type ${block.type} for message cascade:`, e);
      }
    };

    // Start the search from all top-level blocks in this workspace
    workspace.getTopBlocks().forEach(block => {
      findBlocksToDelete(block, normalizedMessageName);
    });

    // Use a Set to ensure uniqueness before disposing
    const uniqueBlocksToDelete = [...new Set(blocksToDelete)];

    // Delete the identified blocks in reverse order
    console.log(`Disposing ${uniqueBlocksToDelete.length} blocks via message cascade in workspace ${workspace.id}`);
    uniqueBlocksToDelete.reverse().forEach(block => {
      if (!block.isDisposed()) { // Check if not already disposed
        try {
          console.log(`  Deleting block ${block.id} of type ${block.type}`);
          block.dispose(true); // true to delete connected blocks
        } catch (e) {
          console.error(`  Error deleting block ${block.id}:`, e);
        }
      } else {
        console.log(` Block ${block.id} was already disposed (message cascade).`);
      }
    });
    // End of modified function scope
  }

  // Add these two new methods
  handleTermination(tabId: number, sessionId: string, message: string): void {
    console.log(`Handling termination for tab ${tabId}, session ${sessionId}`);
    const tab = this.tabs.find(t => t.id === tabId);
    if (tab) {
      tab.isPlaying = false;
    }

    // Append final message
    // const finalMessage = `\\n--- ${message} (Session: ${sessionId}) ---\\n`; // Commented out
    // this.consolesOutput.set(tabId.toString(), (this.consolesOutput.get(tabId.toString()) ?? '') + finalMessage); // Commented out
    if (this.selectedTabId === tabId) {
      // Still update the display even if no message is added, to potentially show last log lines
      this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) ?? '';
      if (this.autoScrollEnabled) {
        setTimeout(() => this.scrollToBottom(), 100);
      }
    }

    // Clean up resources
    this.websockets.get(tabId.toString())?.unsubscribe();
    this.websockets.delete(tabId.toString());
    this.consolesSessions.delete(tabId.toString());

    // Optionally close the service connection (though backend likely does)
    // this.consolesServices.get(tabId.toString())?.closeConnection();

    // Trigger change detection if needed for UI updates (e.g., button states)
    this.changeDetectorRef.detectChanges();
  }

  handleError(tabId: number, sessionId: string, errorDetail: string): void {
    console.error(`Handling error for tab ${tabId}, session ${sessionId}: ${errorDetail}`);
    const tab = this.tabs.find(t => t.id === tabId);
    if (tab) {
      tab.isPlaying = false;
    }

    // Append error message
    const errorMessage = `\n--- ERROR: ${errorDetail} (Session: ${sessionId}) ---\\n`;
    this.consolesOutput.set(tabId.toString(), (this.consolesOutput.get(tabId.toString()) ?? '') + errorMessage);
    if (this.selectedTabId === tabId) {
      this.currentDisplayedConsoleOutput = this.consolesOutput.get(tabId.toString()) ?? '';
      if (this.autoScrollEnabled) {
        setTimeout(() => this.scrollToBottom(), 100);
      }
    }

    // Clean up resources
    this.websockets.get(tabId.toString())?.unsubscribe();
    this.websockets.delete(tabId.toString());
    this.consolesSessions.delete(tabId.toString());

    // Optionally close the service connection
    // this.consolesServices.get(tabId.toString())?.closeConnection();

    // Trigger change detection
    this.changeDetectorRef.detectChanges();
  }

  // Getter to check the status of the selected tab
  get selectedTabIsPlaying(): boolean {
    if (this.selectedTabId === null) {
      return false;
    }
    const tab = this.tabs.find(t => t.id === this.selectedTabId);
    return tab ? tab.isPlaying : false;
  }

  // Getter to check if any tab can be played
  get canPlayAny(): boolean {
    return this.tabs.length > 0 && this.tabs.some(tab => !tab.isPlaying);
  }

  // Getter to check if any tab can be stopped
  get canStopAny(): boolean {
    return this.tabs.length > 0 && this.tabs.some(tab => tab.isPlaying);
  }
}

// Move helper functions outside the class and export them
export function hasValidChain(block: Blockly.Block | null, childBlockType: string): boolean {
  if (!block) return false;

  if (block.type === childBlockType) {
      // Original check: Check if it's the direct block type
      // Extended check: For blocks like publish/send_request, ensure fields are connected
      if (childBlockType === 'ros2_publish_message' || childBlockType === 'ros_send_request') {
          const hasConnectedFields = block.inputList.some(input =>
        input.name?.startsWith('FIELD_') &&
        input.connection &&
        input.connection.targetBlock()
          );
          if (!hasConnectedFields && block.inputList.some(input => input.name?.startsWith('FIELD_'))) {
             // Has the block but no connected fields where fields exist
             return false;
          }
          // If no FIELD_ inputs exist (e.g. empty message), it's valid
          return true; 
      } else {
          // For other block types, just finding the block is enough
          return true;
      }
  }

  // Check inputs recursively
  for (const input of block.inputList) {
    const child = input.connection?.targetBlock();
    if (hasValidChain(child ?? null, childBlockType)) {
      return true;
    }
  }

  // Also check out the next block connected in chain
  const next = block.nextConnection?.targetBlock();
  if (hasValidChain(next ?? null, childBlockType)) {
    return true;
  }

  return false;
}

export function hasAllFieldsConnected(block: Blockly.Block): boolean {
  if (!block || (block.type !== 'ros_send_request' && block.type !== 'ros2_publish_message')) {
       // Added ros2_publish_message for consistency
      return true; // Or false depending on desired behavior for non-applicable blocks
  }

  const fieldInputs = block.inputList.filter(input => input.name?.startsWith("FIELD_"));

  // If there are no FIELD_ inputs (e.g., a message/service with no fields), it's considered valid.
  if (fieldInputs.length === 0) return true;

  // Verify that all existing FIELD_ inputs have a block connected
  return fieldInputs.every(input => input.connection?.targetBlock());
}

function resetNodePerf() {
  const nodeMarks = [
    'ps_start', 'upload_start', 'upload_end',
    'exec_start', 'exec_end', 'ws_wait', 'ws_first',
    'clickClient', 'srv_first'
  ];
  nodeMarks.forEach(m => performance.clearMarks(m));
  nodeMarks.forEach(m => performance.clearMeasures(m));
}

function safeMeasure(name: string, start: string, end: string) {
  if (!performance.getEntriesByName(name).length) {
    performance.measure(name, start, end);
  }
}


