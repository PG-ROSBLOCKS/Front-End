import { Injectable, ComponentFactoryResolver, ApplicationRef, Injector, EmbeddedViewRef, ComponentRef } from '@angular/core';
import { AlertComponent } from './alert.component';

@Injectable({
  providedIn: 'root'
})
export class AlertService {
  private alertComponentRef: ComponentRef<AlertComponent> | null = null;

  constructor(
    private componentFactoryResolver: ComponentFactoryResolver,
    private appRef: ApplicationRef,
    private injector: Injector
  ) {}

  showAlert(message: string): Promise<boolean> {
    return new Promise((resolve) => {
      const componentFactory = this.componentFactoryResolver.resolveComponentFactory(AlertComponent);
      this.alertComponentRef = componentFactory.create(this.injector);
      this.alertComponentRef.instance.message = message;
      this.alertComponentRef.instance.showCancel = false;
      this.alertComponentRef.instance.ok.subscribe((result: boolean) => {
        this.removeAlert();
        resolve(result);
      });

      this.appRef.attachView(this.alertComponentRef.hostView);
      const domElem = (this.alertComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
      document.body.appendChild(domElem);
    });
  }

  showConfirm(message: string): Promise<boolean> {
    return new Promise((resolve) => {
      const componentFactory = this.componentFactoryResolver.resolveComponentFactory(AlertComponent);
      this.alertComponentRef = componentFactory.create(this.injector);
      this.alertComponentRef.instance.message = message;
      this.alertComponentRef.instance.showCancel = true;
      this.alertComponentRef.instance.ok.subscribe((result: boolean) => {
        this.removeAlert();
        resolve(result);
      });

      this.appRef.attachView(this.alertComponentRef.hostView);
      const domElem = (this.alertComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
      document.body.appendChild(domElem);
    });
  }

  private removeAlert() {
    if (this.alertComponentRef) {
      this.appRef.detachView(this.alertComponentRef.hostView);
      this.alertComponentRef.destroy();
      this.alertComponentRef = null;
    }
  }
}