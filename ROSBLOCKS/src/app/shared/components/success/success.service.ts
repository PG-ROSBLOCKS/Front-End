import { Injectable, ComponentFactoryResolver, ApplicationRef, Injector, EmbeddedViewRef, ComponentRef } from '@angular/core';
import { SuccessComponent } from './success.component';

@Injectable({
  providedIn: 'root'
})
export class SuccessService {
  private successComponentRef: ComponentRef<SuccessComponent> | null = null;

  constructor(
    private componentFactoryResolver: ComponentFactoryResolver,
    private appRef: ApplicationRef,
    private injector: Injector
  ) {}

  showSuccess(message: string): Promise<boolean> {
    return new Promise((resolve) => {
      // Creates fabric del componente SuccessComponent
      const componentFactory = this.componentFactoryResolver.resolveComponentFactory(SuccessComponent);
      // Creates component
      this.successComponentRef = componentFactory.create(this.injector);
      // Asigns message
      this.successComponentRef.instance.message = message;
      // Suscribe to output to resolve the promise
      this.successComponentRef.instance.ok.subscribe((result: boolean) => {
        this.removeSuccess();
        resolve(result);
      });

      // Add the component to the view tree
      this.appRef.attachView(this.successComponentRef.hostView);
      // Get the DOM element of the component
      const domElem = (this.successComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
      // Insert it into the body or another defined container
      document.body.appendChild(domElem);
    });
  }

  private removeSuccess() {
    if (this.successComponentRef) {
      this.appRef.detachView(this.successComponentRef.hostView);
      this.successComponentRef.destroy();
      this.successComponentRef = null;
    }
  }
}