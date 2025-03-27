import { Injectable, ComponentFactoryResolver, ApplicationRef, Injector, EmbeddedViewRef, ComponentRef } from '@angular/core';
import { ErrorsComponent } from './errors.component';

interface ErrorsQueueItem {
  message: string;
  showCancel: boolean;
  resolve: (result: boolean) => void;
}

@Injectable({
  providedIn: 'root'
})
export class ErrorsService {
  private alertComponentRef: ComponentRef<ErrorsComponent> | null = null;
  private queue: ErrorsQueueItem[] = [];
  private isShowing = false;

  constructor(
    private componentFactoryResolver: ComponentFactoryResolver,
    private appRef: ApplicationRef,
    private injector: Injector
  ) {}

  showErrors(message: string): Promise<boolean> {
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

    const componentFactory = this.componentFactoryResolver.resolveComponentFactory(ErrorsComponent);
    this.alertComponentRef = componentFactory.create(this.injector);
    this.alertComponentRef.instance.message = message;
    this.alertComponentRef.instance.showCancel = showCancel;
    this.alertComponentRef.instance.ok.subscribe((result: boolean) => {
      this.removeErrors();
      resolve(result);
      this.isShowing = false;
      // Pequeño retardo para dar tiempo a actualizar la vista
      setTimeout(() => this.processQueue(), 50);
    });

    this.appRef.attachView(this.alertComponentRef.hostView);
    const domElem = (this.alertComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
    document.body.appendChild(domElem);
  }

  private removeErrors() {
    if (this.alertComponentRef) {
      this.appRef.detachView(this.alertComponentRef.hostView);
      this.alertComponentRef.destroy();
      this.alertComponentRef = null;
    }
  }
}
