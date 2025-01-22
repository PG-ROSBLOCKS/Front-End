import { Component } from '@angular/core';

@Component({
  selector: 'app-workspace',
  templateUrl: './workspace.component.html',
  styleUrls: ['./workspace.component.css']
})
export class WorkspaceComponent {
  buttonItems: { 
    buttonLabel: string; 
    items: string[]; 
    isListVisible: boolean;
  }[] = [
    { buttonLabel: 'nodes', items: ['Elemento 1.1', 'Elemento 1.2', 'Elemento 1.3'], isListVisible: false },
    { buttonLabel: 'Services', items: ['Elemento 2.1', 'Elemento 2.2', 'Elemento 2.3'], isListVisible: false },
    { buttonLabel: 'Topics', items: ['Elemento 3.1', 'Elemento 3.2', 'Elemento 3.3'], isListVisible: false },
    { buttonLabel: 'etc', items: ['Elemento 2.1', 'Elemento 2.2', 'Elemento 2.3'], isListVisible: false }
  ];

  // Función para alternar la visibilidad de las listas
  toggleList(index: number) {
    this.buttonItems[index].isListVisible = !this.buttonItems[index].isListVisible;
  }



  tabs: { name: string, id: number }[] = [];
  selectedTabId: number | null = null;
  selectedTabName: string | null = null;

  addTab() {
    const newTabId = this.tabs.length + 1;
    this.tabs.push({ name: `Nodo ${newTabId}`, id: newTabId });
    this.selectTab(newTabId);
  }

  selectTab(tabId: number) {
    this.selectedTabId = tabId;
    const selectedTab = this.tabs.find(tab => tab.id === tabId);
    this.selectedTabName = selectedTab ? selectedTab.name : null;
  }

  changeTabName(tabId: number, newName: string) {
    const tab = this.tabs.find(tab => tab.id === tabId);
    if (tab) {
      tab.name = newName;
    }
  }

  playTab(tabId: number) {

  }

  deleteTab(tabId: number) {
    this.tabs = this.tabs.filter(tab => tab.id !== tabId);
    if (this.selectedTabId === tabId) {
      this.selectedTabId = this.tabs.length > 0 ? this.tabs[0].id : null;
    }
  }
}
