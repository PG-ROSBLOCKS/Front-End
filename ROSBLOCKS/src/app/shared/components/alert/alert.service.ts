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
      // Creates fabric del componente AlertComponent
      const componentFactory = this.componentFactoryResolver.resolveComponentFactory(AlertComponent);
      // Creates component
      this.alertComponentRef = componentFactory.create(this.injector);
      // Asigns message
      this.alertComponentRef.instance.message = message;
      // Suscribe to output to resolve the promise
      this.alertComponentRef.instance.ok.subscribe((result: boolean) => {
        this.removeAlert();
        resolve(result);
      });

      // Add the component to the view tree
      this.appRef.attachView(this.alertComponentRef.hostView);
      // Get the DOM element of the component
      const domElem = (this.alertComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
      // Insert it into the body or another defined container
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