import { Component, ElementRef, ViewChild } from '@angular/core';
import { Location } from '@angular/common';
import { Router } from '@angular/router';

@Component({
  selector: 'app-rosblocks-help',
  templateUrl: './rosblocks-help.component.html',
  styleUrls: ['./rosblocks-help.component.css']
})
export class RosblocksHelpComponent {

  constructor(
    private location: Location, // Imported from @angular/common
    private router: Router
  ) {}


  dropdowns: { [key: number]: boolean } = {};
  selectedVideo: string | null = null;

  onSearch(event: any): void {
    console.log('searching:', event.target.value);
  }

  toggleDropdown(id: number): void {
    this.dropdowns[id] = !this.dropdowns[id];
  }

  selectVideo(url: string) {
    this.selectedVideo = url;
    const videoData = this.videoSections.find(video => video.url === url);
    if (videoData) {
      this.selectedTitle = videoData.title;
      this.selectedDescription = videoData.description;
    } else {
      this.selectedTitle = '';
      this.selectedDescription = [];
    }
  }

  goBack(): void {
    // `navigationId` > 1 means there is a previous route in Angular’s history
    if (window.history.state?.navigationId > 1) {
      this.location.back();
    } else {
      this.router.navigate(['/workspace']); // Fallback to landing
    }
  }

    
  videoSections = [
    {
      url: 'https://www.youtube.com/embed/3n1agY5XYOk?si=QdJwRjAO0jIiE1xs',
      title: 'Introducción a ROSBlocks',
      description: [
`ROSBlocks es una innovadora herramienta de aprendizaje en línea diseñada para enseñar a los usuarios sobre el sistema operativo de robótica ROS 2. Se expone que ROSBlocks utiliza un enfoque basado en bloques, similar a plataformas educativas como Scratch y Lego Mindstorms, donde cada bloque representa un segmento de código.
      
Esta metodología permite a los usuarios, especialmente a jóvenes y principiantes en robótica, aprender conceptos importantes como el funcionamiento de nodos, tópicos, servicios y mensajería en ROS 2. A través de la conexión de bloques, los estudiantes pueden generar un código en Python que es ejecutable en un entorno de ROS 2.`,
`👩‍🎓 Público objetivo:
ROSBlocks está diseñada para cualquier persona interesada en la robótica y el aprendizaje de ROS 2, promoviendo el aprendizaje por la robótica.
      
🛠️ Metodología de bloques:
La metodologóia de enseñanza por bloques de programación permite a los principiantes visualizar y entender la lógica detrás del código, facilitando el aprendizaje conceptual.
      
🌍 Fomento de experiencias prácticas:
ROSBlocks promueve el aprendizaje a través de la práctica activa, lo cual es esencial en la educación técnica.
      
⏳ Curva de aprendizaje acelerada:
La programación por bloques ofrece un nivel de abstracción de código que ayuda a comprender con mayor facilidad como funciona la lógica programática de ROS 2, lo cual permite dominar conceptos clave en robótica de forma más rápida e intuitiva.

🤖 Atractivo para educadores:
ROSBlocks también es una herramienta útil para docentes que buscan nuevas formas de enseñar programación y robótica en sus clases, potenciando la experiencia educativa integral.`
      ]
    },
    {
      url: 'https://www.youtube.com/embed/QEOcgvTbHLM?si=LuHonW5Bu9qf9RWL',
      title: 'Funcionalidades básicas',
      description: [
`El tutorial presenta una introducción a las funcionalidades básicas de la herramienta donde los usuarios aprenden a crear nodos, nombrarlos, y gestionar los bloques que los componen. También se explica cómo duplicar, iniciar y detener nodos, visualizar su información desde consolas, explorar las secciones de bloques disponibles y cómo se genera el código Python correspondiente a cada acción. Finalmente, se introduce el uso del simulador TurtleSim para observar el comportamiento del robot en diferentes escenarios.`,
`🌐 Funcionamiento del Sistema de Nodos:
Cada nodo opera como un proceso independiente dentro de ROS 2. Comprender esta arquitectura es esencial para construir aplicaciones robóticas modulares y eficientes.

💡 Interacción Visual en ROSBlocks: 
La interfaz permite arrastrar bloques de manera visual para componer código.

⚙️ Duplicar y Eliminar Nodos:
Duplicar nodos ahorra tiempo al desarrollar estructuras repetitivas, mientras que eliminarlos permite depurar o reorganizar el proyecto con facilidad.

📜 Código en Python:
Cada bloque genera automáticamente código en Python. Este código puede ser exportado y ejecutado en otros entornos.
      
🐢 Simulador TurtleSim:
El simulador ayudando a entender cómo responde a las instrucciones codificadas en distintos escenarios.
      
⚠️ Gestión de Consolas por Nodo:
Cada nodo tiene su consola dedicada que permite monitorear su comportamiento.
      
🎛️ Funcionalidades de Guardado y Carga:
ROSBlocks permite guardar y cargar proyectos.`,
      ]
      
    },
    {
      url: 'https://www.youtube.com/embed/yhh464-fKWI?si=F6I4RLbClCGigqbN',
      title: 'Correcta gestión de recursos y bloques',
      description: [
`Este video se enfoca en la gestión eficaz de los recursos y el uso adecuado de los bloques en la construcción de nodos dentro de ROSBlocks.
      
Se explica cómo cada nodo o pestaña representa un proceso que consume recursos del sistema, y cómo su uso descontrolado puede afectar negativamente el rendimiento.
      
También se muestra que ciertos bloques requieren de otros bloques complementarios para poder ejecutarse correctamente.
      
Se enfatiza la importancia de pausar la ejecución al hacer cambios en los bloques y prestar atención a los errores y advertencias generados por el sistema, los cuales guían el desarrollo correcto del proyecto.`,
      
`📈 Gestión de recursos y rendimiento:
Limitar la cantidad de nodos o pestañas activas mejora la eficiencia del sistema y garantiza una experiencia más fluida.
      
🔍 Procesos y analogías prácticas:
La comparación con pestañas en un navegador ayuda a visualizar cómo los nodos afectan el uso de CPU y memoria.
      
🔗 Estructura lógica de bloques:
Algunos bloques no funcionan de forma aislada. Entender las dependencias es clave para armar proyectos funcionales y coherentes.
      
⚡ Resolución activa de errores:
El sistema notifica problemas, lo que permite tener retroalimentación de que corregir para tener una ejecución correcta del código.
      
🛑 Necesidad de pausar y recompilar:
Modificar bloques mientras se ejecuta un nodo hace que este se pause. Es necesario reiniciar nuevamente el nodo para que los cambios surtan efecto.`
      ]
      
    },
    {
      url: 'https://www.youtube.com/embed/OuZQJktaxN8?si=fc4FCzmqNSJLZ0-7',
      title: 'Hola mundo',
      description: [
`Este tutorial introduce a los usuarios en las funciones básicas de programación utilizando bloques en el entorno RosBlocks.

El video explica el papel fundamental de los nodos dentro del sistema ROS 2, mostrando cómo cada nodo representa un proceso que ejecuta instrucciones definidas visualmente. A través de la unión de bloques, se genera código que luego puede ser ejecutado por el sistema.

Se guía al usuario para crear un nodo llamado “ejercicio uno” y desarrollar un ejemplo sencillo: imprimir el texto “Hola Mundo” en la consola usando bloques. También se presenta el uso de bloques para imprimir valores numéricos o concatenar textos, fomentando una exploración activa del entorno.`,

`🔗 Importancia de los nodos:
Cada nodo funciona como una unidad de ejecución en ROS. Comprender su función es esencial para construir programas robóticos efectivos y modulares.

👩‍💻 Uso del bloque “print”:
Mostrar resultados en la consola mediante el bloque “print” permite verificar el comportamiento del código, una técnica esencial para cualquier programador.

🔠 Concatenación de cadenas:
Se presenta cómo unir textos dinámicamente con bloques de concatenación, una habilidad básica pero útil en múltiples contextos de programación.

📊 Flujo de ejecución:
Se introduce el bloque “init” como punto de partida del programa, ayudando a los usuarios a entender el orden de ejecución dentro de los nodos.
`
]
    },
    {
      url: 'https://www.youtube.com/embed/ywTdRU6jq0M?si=EptUYYzOy-rMX6iB',
      title: 'Operadores matemáticos',
      description: [
  `En el tutorialse presenta un enfoque práctico sobre el uso de operadores matemáticos en la programación con bloques.

Se introducen operaciones básicas como suma, resta, multiplicación, división y exponenciación, junto con funciones más avanzadas como raíces cuadradas, funciones trigonométricas y generación de números aleatorios. Mediante ejemplos, como la suma de 1 + 2, se demuestra cómo utilizar estos bloques y visualizar los resultados en la consola, motivando a los usuarios a experimentar por su cuenta.`,

  `🧠 Conceptos clave del tutorial:

📊 Fundamentos de la programación matemática:
Comprender cómo se utilizan los operadores matemáticos es esencial para cualquier programador, ya que permiten resolver problemas y expresar lógicas numéricas dentro del código.

🚀 Ejercicios prácticos:
El tutorial refuerza los conceptos al mostrar resultados en tiempo real, como calcular sumas o generar valores aleatorios.

🎯 La consola como herramienta de feedback:
Observar la salida de los cálculos en la consola permite validar el comportamiento del código al instante, facilitando el proceso de depuración y comprensión.`
]      
    },
    {
      url: 'https://www.youtube.com/embed/sKIScjO1Gn4?si=0eHLwqqwnjYcnrJb',
      title: 'Condicionales',
      description: [
  `En el tutorial se introduce el uso de operadores condicionales en programación mediante bloques.

Se presenta la lógica de instrucciones condicionales como “if” y “if-else”, que permiten ejecutar bloques de código según el cumplimiento o no de una condición.

Con ejemplos prácticos, se demuestra cómo comparar números (por ejemplo, si dos valores son iguales), y cómo usar operadores lógicos para manejar distintos resultados. También se aborda el concepto de negación lógica, mostrando cómo una condición falsa puede invertir el flujo de ejecución.`,

  `🧠 Conceptos clave del tutorial:

📈 La importancia de los condicionales:
Los condicionales permiten que un programa tome decisiones basadas en información evaluada en tiempo real. Son esenciales para introducir lógica dinámica en cualquier aplicación.

📏 Comparaciones directas:
Comparar valores como “¿1 es igual a 2?” ayuda a entender de forma clara cómo se implementa la lógica básica en un flujo programático.

🧩 Estructuras de control:
Instrucciones como “if” y “if-else” permiten que el flujo del programa se bifurque según condiciones, lo cual es la base de algoritmos más avanzados.

❌ Negación y lógica inversa:
El operador de negación ofrece flexibilidad para invertir condiciones y tomar decisiones en escenarios más complejos.`
]
    },
    {
      url: 'https://www.youtube.com/embed/HkH_jvJuW6I?si=pH--FQet96HGhy9b',
      title: 'Variables',
      description: [
  `En el tutorial se introduce el concepto y uso de las variables en la programación mediante bloques.

Se muestra cómo crear una variable llamada “ítem” con un valor de 10, imprimir su contenido en consola y luego crear otra variable llamada “mundo” con el mismo valor. Posteriormente, se realiza una suma entre ambas, ilustrando los conceptos fundamentales de asignación, uso y manipulación de variables en un entorno visual.`,

  `🧠 Conceptos clave del tutorial:

🎯 Las variables son fundamentales en programación:
Permiten almacenar, modificar y reutilizar información dentro de un programa, asimismo, son la base para construir lógica dinámica y resolver problemas de forma estructurada.

📐 Operadores matemáticos en programación:
Aplicar operadores sobre variables permite resolver tareas más complejas y simula escenarios reales de programación.`
]
    },
    {
      url: 'https://www.youtube.com/embed/mDGx1dLg2pM?si=pQcpR53LKwg-lLUE',
      title: 'Bucles',
      description: [
  `En el tutoria de la sección dos de ROSBlocks, se aborda el uso de bucles como estructuras fundamentales para repetir instrucciones en la programación con bloques.

El ejercicio inicia introduciendo un bucle que repite la misma acción diez veces, con una pausa entre cada iteración para mostrar cómo se imprime el número 15 secuencialmente.

El tutorial avanza con el uso de una variable que define la cantidad de repeticiones del bucle, mostrando cómo se puede controlar el flujo según los valores definidos por el usuario. También se presenta el bucle “while”, el cual continúa ejecutándose mientras se cumpla una condición lógica, y se advierte sobre el riesgo de crear bucles infinitos.

Finalmente, se explora el bucle de conteo, donde se inicializa una variable y se imprime su valor dentro de un rango específico.`,

  `🧠 Conceptos clave del tutorial:

🔄 Uso de bucles en programación:
Los bucles permiten ejecutar bloques de código múltiples veces, facilitando tareas repetitivas y reduciendo la necesidad de escribir instrucciones duplicadas.

⏱️ Ejemplo práctico de repetición:
Repetir la impresión de un valor con pausas visibles demuestra cómo el flujo de ejecución puede ser controlado temporalmente y ayuda a visualizar el comportamiento del código.

📊 Variables para control de iteraciones:
Utilizar variables para determinar cuántas veces se ejecuta un bucle ofrece flexibilidad y permite parametrizar la ejecución.

⚠️ Precaución con bucles infinitos:
Los bucles “while” pueden ejecutarse sin fin si no se controlan correctamente. El tutorial advierte sobre su uso responsable para evitar consumo excesivo de recursos.

🔢 Bucle de contar:
El incremento de una variable en cada iteración permite recorrer rangos y realizar acciones dependientes del número actual de la repetición.`
]
    },
    {
      url: 'https://www.youtube.com/embed/jqcUW2ZM0Ik?si=WNMpNRw8ZpFcF76m',
      title: '¿Cómo funcionan los tópicos?',
      description: [
        `En eltutorial se abordan los fundamentos de los tópicos como mecanismo de comunicación entre nodos en un sistema distribuido.

Se introduce el concepto del tópico como un contenedor de mensajes donde un nodo publicador envía datos, y un nodo suscriptor los recibe. Esta arquitectura permite una transmisión eficiente y desacoplada de información.

El video presenta un diagrama explicativo que ilustra cómo los datos fluyen desde el nodo que publica hacia aquellos que están suscritos, mostrando una estructura escalable y clara. Además, se destaca la incorporación de música en el video como un recurso para mejorar la experiencia de aprendizaje.`,

  `🧠 Conceptos clave del tutorial:

🔄 Funcionalidad del Tópico:
Los tópicos son canales de comunicación que permiten que múltiples nodos intercambien mensajes de manera organizada, sin necesidad de conexiones directas entre ellos.

📜 Roles de Nodos:
El nodo publicador emite datos hacia el tópico, mientras que el suscriptor está encargado de recibirlos. Ambos deben funcionar de forma coordinada para lograr una comunicación efectiva.`
      ]
    },
    {
      url: 'https://www.youtube.com/embed/HYLY4aPUVY8?si=HnrKPWu2j1XeVSY_',
      title: 'Ejercicio de tópicos',
      description: [
        `En el tutorial se presenta un ejercicio práctico del patrón publicador-suscriptor.

El tutorial guía a los usuarios en la creación de dos nodos: uno para publicar mensajes y otro para suscribirse a ellos. Se crean dos espacios de trabajo, “publicador” y “suscriptor”, y se explica cómo asignar roles específicos a cada nodo.

Se muestra la importancia de usar el mismo nombre de tópico en ambos nodos para establecer la comunicación, y se demuestra cómo múltiples suscriptores pueden conectarse a un único publicador. También se evidencia que si los nombres de los tópicos no coinciden, la comunicación no ocurre.

El video concluye enseñando cómo visualizar los tópicos activos del sistema para monitorear y verificar el funcionamiento correcto de los nodos.`,

  `🧠 Conceptos clave del tutorial:

📊 Configuración de nodos es crucial:
Asignar correctamente los roles de publicador y suscriptor y asegurarse de que ambos usen el mismo nombre de tópico garantiza una comunicación exitosa.

📞 Importancia del tipo de datos:
Especificar correctamente el tipo de mensaje (por ejemplo, string) permite que los datos sean entendidos por los suscriptores.

⏲️ Bucles para repetición de mensajes:
El uso de bucles permite enviar múltiples mensajes de manera controlada, ajustando la frecuencia y cantidad según las necesidades del sistema.

📥 Flexibilidad con múltiples suscriptores:
Un mismo tópico puede ser escuchado por varios nodos suscriptores.

📈 Visualización de tópicos activos:
ROSBlocks permite ver los tópicos activos en tiempo real, lo cual es una herramienta clave para diagnosticar fallos o verificar que los nodos están correctamente conectados.`
      ]      
    },
    {
      url: 'https://www.youtube.com/embed/kfTGi7bi8Lg?si=ujLwSrLUQOdRO87g',
      title: '¿Cómo funcionan los servicios?',
      description: [
  `En el tutorial se introduce el modelo de comunicación cliente-servidor.

En este modelo, un nodo cliente envía una solicitud a un servicio, esperando una respuesta activa. Esto significa que la comunicación requiere que el servidor esté siempre activo y disponible para responder, estableciendo una relación directa y dependiente entre ambos componentes.

El tutorial destaca cómo esta interacción sincrónica es fundamental en aplicaciones que requieren respuestas inmediatas o transacciones completas. Se aclara que, a diferencia del publicador que puede enviar su mensaje y desconectarse, el cliente debe esperar la respuesta del servidor para continuar su ejecución.`,

  `🧠 Conceptos clave del tutorial:

📊 Interacción Cliente-Servidor:
El cliente inicia la comunicación y espera una respuesta directa del servidor. Esta relación sincrónica es la base de la mayoría de aplicaciones web y móviles modernas.

🛠️ Dependencia del Servidor:
El servidor debe estar operativo continuamente para responder a las solicitudes. Si el servidor falla o se desconecta, el cliente queda inactivo, afectando la experiencia del usuario.

⚖️ Comparativa con Publicador-Suscriptor:
En el modelo cliente-servidor, el flujo de comunicación depende completamente del servidor, a diferencia del modelo publicador-suscriptor que es más desacoplado y flexible.`
      ]
    },
    {
      url: 'https://www.youtube.com/embed/bYQr1gWEfBg?si=0oSj-T62FgN0vMjP',
      title: 'Ejercicio de servicios',
      description: [
        `En el tutorial se desarrolla un ejercicio práctico para construir un modelo cliente-servidor utilizando un servicio llamado “suma”.

El proceso comienza con la creación del servicio, dividido en una sección de solicitud (request), donde se ingresan dos números, y una sección de respuesta (response), donde se devuelve el resultado de la operación. Se construyen dos nodos: uno actúa como servidor y el otro como cliente.

El servidor espera solicitudes y responde con la suma de los dos números proporcionados. Si no responde dentro de un tiempo definido, el cliente muestra un mensaje de espera. También se muestra cómo imprimir el resultado en la consola y realizar múltiples repeticiones de la operación para observar la dinámica completa del servicio.`,

  `🧠 Conceptos clave del tutorial:

🔍 Entendimiento del servicio ROS 2:
Los servicios permiten establecer comunicaciones bidireccionales en ROS 2, facilitando la ejecución de operaciones específicas bajo demanda entre nodos.

🌐 Interacción asíncrona:
El manejo del tiempo de espera entre cliente y servidor asegura una interacción fluida sin bloquear el sistema, previniendo cuelgues durante el intercambio de datos.`
      ]      
    },
    {
      url: 'https://www.youtube.com/embed/FjxuTH_zGeA?si=SfLwO8g3ZjhXElz6',
      title: 'Ejercicio de custom messages',
      description: [
  `En el tutorial se enseña cómo crear mensajes personalizados (custom messages).

El tutorial inicia con una revisión del menú de bloques disponibles para definir tipos de variables y estructuras de mensaje. A continuación, se guía al usuario en la creación de su propio tipo de mensaje, que representará un sensor de temperatura, y en la configuración de un publicador que emite datos de temperatura generados aleatoriamente.

Paralelamente, se construye un nodo suscriptor capaz de recibir esos mensajes y mostrar los datos en la consola. Se resalta la importancia de mantener coherencia en los nombres de tópicos y sensores para asegurar la correcta comunicación entre nodos. El video concluye con una demostración funcional del envío y recepción de mensajes personalizados.`,

  `🧠 Conceptos clave del tutorial:

📋 Creación de mensajes personalizados:
Diseñar estructuras de datos a medida permite adaptar la comunicación a las necesidades específicas del sistema.
`
      ]
    },
    {
      url: 'https://www.youtube.com/embed/4yl0FtIFwwQ?si=x56pXq1Ql5SIYNpP',
      title: 'Introducción a Turtlesim',
      description: [
  `En este módulo se introduce el comportamiento de un robot simulado con Turtlesim.

Este simulador es una herramienta educativa ideal para practicar conceptos fundamentales de robótica en un entorno seguro, visual e interactivo, sin necesidad de hardware físico.`, 

  `🧠 Conceptos clave del tutorial:

🎯 Interacción con el entorno:
Turtlesim permite interactuar con un espacio virtual que emula cómo se comportaría un robot real, facilitando la comprensión de acciones como desplazamientos, giros y trayectorias.

🌍 Versatilidad en simulaciones:
Los usuarios pueden controlar una o varias tortugas, diseñar movimientos personalizados y simular diferentes comportamientos que reflejan situaciones reales en robótica.

👩‍🏫 Aplicación en la educación:
Turtlesim es una herramienta valiosa para docentes que desean enseñar robótica de forma visual, clara y atractiva, integrando teoría y práctica de forma didáctica.`
      ]
    },
    {
      url: 'https://www.youtube.com/embed/8F5Q3jEI7vc?si=JjuMltC4XHJDD3ob',
      title: 'Funcionalidades, mapas predeterminados y personalizados',
      description: [
  `En este tutorial se explican las funcionalidades del simulador, incluyendo el control del movimiento de la tortuga, el uso de tópicos, y la personalización de mapas.

Se introduce el tópico encargado de definir la velocidad lineal y angular de la tortuga, lo que permite controlar su movimiento en el entorno. También se abordan otros tópicos que permiten cambiar el color del pincel, el grosor del trazo y pausar la tortuga. 

Además, se presenta la posibilidad de utilizar mapas predeterminados (vacío, con obstáculos y laberinto), así como crear mapas personalizados mediante matrices binarias. El video concluye con una aclaración sobre la regeneración aleatoria de la tortuga en cada reinicio del simulador, lo que añade dinamismo a las pruebas realizadas.`,

  `🧠 Conceptos clave del tutorial:

📊 Entendiendo los tópicos:
Dominar el uso de tópicos en ROS es esencial para interactuar con la tortuga, ya que cada aspecto de su comportamiento se controla a través de estos canales.

🐍 Funcionalidad de “cmd_vel”:
Este tópico específico permite modificar la velocidad de desplazamiento y rotación, ajustando el comportamiento dinámico de la tortuga en tiempo real.

🎭 Interacción con colores:
Cambiar el color y el grosor del trazo de la tortuga no solo agrega un elemento visual atractivo, sino que también permite representar trayectorias.

🧩 Creación de mapas propios:
Usar matrices de ceros y unos para definir mapas personalizados abre posibilidades para diseñar entornos únicos, fomentando la creatividad y el aprendizaje experimental.

🔄 Reinicio del simulador:
Al reiniciar el entorno, se genera una tortuga nueva, lo que impide configurar una tortuga fija, pero aporta variedad a los ejercicios.

⚖️ Restricciones en mapas:
Los mapas deben ser cuadráticos, lo que requiere planificación cuidadosa para evitar errores y aprovechar al máximo el espacio disponible.

📏 Simulación de colisiones:
Los elementos de los mapas permiten simular obstáculos y restricciones de movimiento, lo cual es fundamental para el aprendizaje de navegación y planificación de trayectorias en robótica.`
      ]
    },
    {
      url: 'https://www.youtube.com/embed/jofV0_vv3lQ?si=bj6MCsvO1OrdD7Uk',
      title: 'Movimientos de la tortuga',
      description: [
  `En el tutorial se exploran las diferentes formas de controlar el movimiento de la tortuga en el simulador, desde su inicialización hasta la personalización de su trazo.

El tutorial comienza explicando la importancia del bloque de inicialización, el cual debe ejecutarse antes de enviar cualquier comando a la tortuga. Luego, mediante el bloque “publishtwist from turtle”, se muestra cómo ajustar la velocidad lineal y angular para definir movimientos precisos.

También se enseña cómo teletransportar a la tortuga a una posición específica, cómo activar o desactivar el lápiz, y cómo cambiar el color y grosor del trazo para obtener un mayor control visual sobre sus desplazamientos. Se abordan además las rotaciones de la tortuga y su aplicación práctica dentro del entorno de simulación.`,

  `🧠 Conceptos clave del tutorial:

📜 Bloques esenciales para el movimiento:
La ejecución siempre comienza con el bloque de inicialización, asegurando que la tortuga esté lista para interpretar comandos correctamente.

🔄 Control preciso del movimiento:
El bloque “publish twist from turtle” permite definir la dirección y velocidad del desplazamiento.

🎨 Personalización visual:
Modificar el color y el grosor del lápiz permite visualizar los recorridos de forma personalizada.

📍 Teletransportación controlada:
Establecer coordenadas específicas para mover la tortuga ayuda a reforzar conceptos de posicionamiento espacial y lógica de coordenadas en programación.

⏳ Uso dinámico del lápiz:
Activar y desactivar el lápiz en momentos específicos evita trazos no deseados y permite un mayor control sobre cuándo y cómo se representan los movimientos.

📈 Rotación efectiva:
Rotar la tortuga en grados específicos introduce conceptos de transformación espacial.`
      ]      
    },
    {
      url: 'https://www.youtube.com/embed/7EYOGoymTm0?si=Xav7hgLQ_FS8_UD3',
      title: 'Exportar el proyecto de ROSBlocks y probarlo en Ubuntu',
      description: [
`En este tutorial  se guía al usuario a través del proceso de exportación de un proyecto para su ejecución en una máquina con Linux, específicamente Ubuntu 20.04.

Utilizando Turtlesim como ejemplo práctico, el tutorial muestra cómo crear un nuevo nodo llamado “tortuga” y organizar su programación utilizando bloques de inicio, movimiento y giro. También se configuran parámetros como la velocidad lineal, la velocidad angular y temporizadores para secuenciar correctamente los movimientos.

Al finalizar, se explica detalladamente cómo exportar el proyecto desde ROSBlocks, compilar el código, y ejecutar el nodo en el simulador de Ubuntu, logrando así ver en pantalla los resultados del trabajo realizado.`,

  `🧠 Conceptos clave del tutorial:
🌍 Exportación a Linux:
Exportar los proyectos a Ubuntu 20.04 permite ejecutar nodos ROS 2 en un entorno real, profesional y ampliamente utilizado en la industria.

🐍 Ejecución de comandos en terminal:
Aprender a utilizar la terminal de Linux para ejecutar nodos es una habilidad clave que prepara a los estudiantes para entornos de desarrollo reales.`
      ]
      
    }
    
  ];
  selectedTitle = '';
  selectedDescription: string[] = [];

  
}
