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
      // Crear la fábrica del componente AlertComponent
      const componentFactory = this.componentFactoryResolver.resolveComponentFactory(AlertComponent);
      // Crear el componente
      this.alertComponentRef = componentFactory.create(this.injector);
      // Asignar el mensaje
      this.alertComponentRef.instance.message = message;
      // Suscribirse al output para resolver la promesa
      this.alertComponentRef.instance.ok.subscribe((result: boolean) => {
        this.removeAlert();
        resolve(result);
      });

      // Agregar el componente al árbol de vistas
      this.appRef.attachView(this.alertComponentRef.hostView);
      // Obtener el elemento DOM del componente
      const domElem = (this.alertComponentRef.hostView as EmbeddedViewRef<any>).rootNodes[0] as HTMLElement;
      // Insertarlo en el body o en otro contenedor definido
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