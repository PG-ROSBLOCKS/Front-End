# ROSBlocks Frontend

**ROSBlocks** is a web-based visual programming environment built with Angular and powered by Google's Blockly library. It is designed to simplify the development of applications using **ROS 2 (Robot Operating System)**, especially for users who are not yet familiar with Linux, Python, or C++. With ROSBlocks, users can create and control ROS 2 nodes using an intuitive drag-and-drop interface directly from their browser.

This repository contains the **frontend client** for the ROSBlocks platform. It handles user interaction, visual programming with Blockly, communication with the backend, and workspace management.

## Key Features

- Visual programming with Blockly blocks adapted to ROS 2 concepts (publishers, subscribers, services, etc.)
- Real-time code generation in Python for ROS 2
- Live execution and monitoring of ROS 2 nodes
- WebSocket-based communication with the backend
- Multi-workspace environment with autosave and project isolation
- Integrated VNC streaming for graphical visualization of ROS simulations (e.g., `turtlesim`)
- Session management and dynamic backend instance allocation

## Technology Stack

- **Angular** – Web application framework
- **TypeScript** – Strictly-typed JavaScript for app logic
- **HTML5 & CSS3** – UI structure and styling
- **RxJS** – Reactive programming for event and data flow
- **Blockly** – Visual programming library for creating custom ROS 2 blocks
- **WebSocket** – Real-time interaction with backend processes

## Prerequisites

- [Node.js](https://nodejs.org/) (LTS version recommended)
- npm (comes with Node.js)
- Angular CLI (install globally via `npm install -g @angular/cli`)
- A running instance of the **ROSBlocks backend**, which processes code execution and simulation

## Installation Guide (Windows & Ubuntu)

### Windows

1. **Install Node.js and npm:**
   - Download the LTS version from [nodejs.org](https://nodejs.org/en/download/)
   - Run the installer and follow the instructions

2. **Install Angular CLI:**
   ```bash
   npm install -g @angular/cli
   ```

3. **Verify Installation:**
   ```bash
   node -v
   npm -v
   ng version
   ```

### Ubuntu

1. **Update repositories:**
   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. **Install Node.js and npm:**
   ```bash
   curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
   sudo apt install -y nodejs
   ```

3. **Install Angular CLI:**
   ```bash
   sudo npm install -g @angular/cli
   ```

4. **Verify Installation:**
   ```bash
   node -v
   npm -v
   ng version
   ```

## Setup (One-time initialization)

1. **Clone the repository:**
   ```bash
   git clone https://github.com/PG-ROSBLOCKS/Front-End.git
   cd Front-End
   ```

2. **Install project dependencies:**
   ```bash
   npm install
   ```

## Running the Application

To start the development server:

```bash
ng serve
```

Then open your browser and go to:

```
http://localhost:4200
```

Changes made to the source files will automatically reload the application in the browser.

---

If you need help setting up the backend or deploying ROSBlocks in a containerized environment, refer to the backend documentation or contact the maintainers.
