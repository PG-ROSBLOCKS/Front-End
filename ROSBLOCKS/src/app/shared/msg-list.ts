// msg-list.ts
export interface MsgVariable {
    type: string;
    name: string;
}

export interface MsgInfo {
    name: string;
    fields: MsgVariable[];
}

export let msgList: MsgInfo[] = [];
