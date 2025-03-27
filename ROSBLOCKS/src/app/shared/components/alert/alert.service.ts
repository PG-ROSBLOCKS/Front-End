import { Injectable, ComponentFactoryResolver, ApplicationRef, Injector, EmbeddedViewRef, ComponentRef } from '@angular/core';
import { AlertComponent } from './alert.component';

interface AlertQueueItem {
  message: string;
  showCancel: boolean;
  resolve: (result: boolean) => void;
}

@Injectable({
  providedIn: 'root'
})
export class AlertService {
  private alertComponentRef: ComponentRef<AlertComponent> | null = null;
  private queue: AlertQueueItem[] = [];
  private isShowing = false;

  constructor(
    private componentFactoryResolver: ComponentFactoryResolver,
    private appRef: ApplicationRef,
    private injector: Injector
  ) {}

  showAlert(message: string): Promise<boolean> {
    return new Promise((resolve) => {
      this.queue.push({ message, showCancel: false, resolve });
      this.processQueue();
    });
  }

  showConfirm(message: string): Promise<boolean> {
    return new Promise((resolve) => {
      this.queue.push({ message, showCancel: true, resolve });
      this.processQueue();
    });
  }

  private processQueue() {
    if (this.isShowing || this.queue.length === 0) {
      return;
    }

    const { message, showCancel, resolve } = this.queue.shift()!;
    this.isShowing = true;

    const componentFactory = this.componentFactoryResolver.resolveComponentFactory(AlertComponent);
    this.alertComponentRef = componentFactory.create(this.injector);
    this.alertComponentRef.instance.message = message;
    this.alertComponentRef.instance.showCancel = showCancel;
    this.alertComponentRef.instance.ok.subscribe((result: boolean) => {
      this.removeAlert();
      resolve(result);
      this.isShowing = false;
      // Pequeño retardo para dar tiempo a actualizar la vista
      setTimeout(() => this.processQueue(), 50);
    });

    this.appRef.attachView(this.alertComponentRef.hostView);
    const domElem = (this.alertComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
    document.body.appendChild(domElem);
  }

  private removeAlert() {
    if (this.alertComponentRef) {
      this.appRef.detachView(this.alertComponentRef.hostView);
      this.alertComponentRef.destroy();
      this.alertComponentRef = null;
    }
  }
}
