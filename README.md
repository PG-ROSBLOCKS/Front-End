# ROSBlocks Frontend

Este proyecto es una aplicación web moderna desarrollada con Angular que integra la biblioteca Blockly para ofrecer una experiencia de programación visual. La combinación de Angular para el manejo de la interfaz y la lógica de negocio, con Blockly para la creación y manipulación de bloques de programación estructurada como de ROS2, permite crear aplicaciones interactivas, didácticas y altamente escalables.

## Tecnologías

- **Angular**
- **TypeScript**
- **HTML5 & CSS3**
- **Angular CLI**
- **Blockly**: Biblioteca para la programación visual con bloques

## Requisitos

- [Node.js](https://nodejs.org/) (versión LTS recomendada)
- npm (incluido con Node.js).
- Angular CLI (instalar globalmente: `npm install -g @angular/cli`)
- El backend debe estar corriendo previamente para que se le puedan enviar las peticiones y las procese.

# Angular Blockly App

Este proyecto es una aplicación web moderna desarrollada con Angular que integra la biblioteca Blockly para ofrecer una experiencia de programación visual. La combinación de Angular para el manejo de la interfaz y la lógica de negocio, con Blockly para la creación y manipulación de bloques, permite crear aplicaciones interactivas, didácticas y altamente escalables.

## Tecnologías

- **Angular**
- **TypeScript**
- **HTML5 & CSS3**
- **Angular CLI**
- **RxJS**
- **Blockly:** Biblioteca para la programación visual con bloques

## Requisitos e Instalación en Windows/Ubuntu
### Windows

1. **Instalar Node.js y npm:**
   - Descarga el instalador de la versión LTS desde [nodejs.org](https://nodejs.org/en/download/).
   - Ejecuta el instalador y sigue las instrucciones; npm se instalará junto con Node.js.

2. **Instalar Angular CLI:**
   - Abre el símbolo del sistema (cmd) o PowerShell y ejecuta:
     ```bash
     npm install -g @angular/cli
     ```

3. **Verificar la instalación:**
   - Ejecuta los siguientes comandos para confirmar que Node.js, npm y Angular CLI están instalados correctamente:
     ```bash
     node -v
     npm -v
     ng version
     ```

### Ubuntu

1. **Actualizar repositorios:**
   ```bash
   sudo apt update
   sudo apt upgrade
   ```
2. **Instalar Angular y Nodejs:**
   ```bash
   curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
   sudo apt-get install -y nodejs
   ```
3. **Instalar Angular CLI:**
   ```bash
   sudo npm install -g @angular/cli
   ```
4. **Verificar la instalación:**
   ```bash
   node -v
   npm -v
   ng version
   ```
## Instalación (solo se hace una única vez)

1. **Clona el repositorio:**

   ```bash
   git clone https://github.com/PG-ROSBLOCKS/Front-End.git
   cd Front-End
   ```
2. **Instalar las dependencias:**
   ```bash
   npm install
   ```
## Iniciar el proyecto de angular
Para iniciar la aplicación en modo desarrollo, ejecuta:
   ```bash
   ng serve
   ```
Abre http://localhost:4200 en tu navegador para ver la aplicación. Los cambios se reflejarán automáticamente al guardar.
