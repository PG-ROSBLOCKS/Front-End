// srv-list.ts
export interface SrvVariable {
    type: string;
    name: string;
  }
  
  export interface SrvInfo {
    name: string;
    variables: {
      request: SrvVariable[];
      response: SrvVariable[];
    };
  }
  
  export let srvList: SrvInfo[] = [];
  