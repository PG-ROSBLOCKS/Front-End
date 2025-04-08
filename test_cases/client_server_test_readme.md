# Casos de Prueba para Cliente/Servidor en ROS2

Este documento describe los casos de prueba para los bloques de cliente y servidor en ROS2, diseñados para el proyecto ROSBLOCKS.

## Descripción General

Los casos de prueba aquí presentados están diseñados para verificar el funcionamiento de los servicios de ROS2, tanto desde el lado del cliente como del servidor, a través de la interfaz de ROSBLOCKS. Las pruebas comprueban diferentes tipos de datos y operaciones, desde análisis de texto hasta cálculos matemáticos.

## IMPORTANTE: Problemas Detectados

Se han identificado problemas de compatibilidad con algunos bloques. Específicamente:

- Los archivos que usan `ros2_create_client` y `ros2_create_server` no se cargan correctamente
- Al revisar el código fuente, se confirma que los tipos correctos a utilizar son `ros_create_client` y `ros_create_server` (sin el "2")
- Las solicitudes de cliente deben usar `ros_send_request` en lugar de `ros2_send_request`

**Todos los nuevos archivos de prueba han sido creados siguiendo esta convención correcta.**

## Objetivos de Prueba

1. Verificar la definición correcta de mensajes de servicio (bloques de solicitud y respuesta)
2. Comprobar que los clientes pueden enviar solicitudes correctamente formateadas
3. Validar que los servidores pueden procesar las solicitudes y generar respuestas adecuadas
4. Probar diferentes tipos de datos y estructuras en las comunicaciones cliente-servidor
5. Validar el manejo de errores y condiciones excepcionales

## Casos de Prueba

### Test Case 1: Cliente/Servidor Integrado - Analizador de Texto
**Archivo:** `complete_client_server_test.rosblocks`

Este caso de prueba incluye tanto el cliente como el servidor en un mismo archivo, permitiendo probar toda la cadena de comunicación. El servicio analiza textos realizando diferentes operaciones como contar palabras, caracteres o verificar si son palíndromos.

**Nota importante:** Utiliza los tipos de bloques correctos `ros_create_client` y `ros_create_server` con sus respectivos métodos.

### Test Case 2: Servidor de Calculadora Básica
**Archivo:** `basic_service_test.rosblocks`

Un caso simplificado que implementa solo un servidor de calculadora que puede realizar operaciones básicas (suma, resta, multiplicación y división). Útil para verificar la funcionalidad del servidor de forma aislada.

### Test Case 3: Cliente de Calculadora
**Archivo:** `basic_client_test.rosblocks`

Cliente que solicita operaciones a un servidor de calculadora. Implementa un bucle para probar diferentes operaciones matemáticas y manejo de errores.

### Test Case 4: Servidor - Planificador de Rutas
**Archivo:** `server_test1_path_planner.rosblocks`

Servidor que calcula la mejor ruta entre dos puntos, ofreciendo diferentes tipos de trayectorias (directa, diagonal, manhattan) y determinando automáticamente cuál es la más eficiente. Devuelve la longitud de la ruta, tipo y número de puntos de paso.

### Test Case 5: Cliente - Planificador de Rutas
**Archivo:** `client_test_path_planner.rosblocks`

Cliente que solicita cálculos de rutas al servidor `PathPlanner`. Utiliza bucles para probar diferentes puntos de inicio y destino, y muestra la información sobre las rutas calculadas.

**Nota**: Utiliza el tipo correcto `ros_create_client` y `ros_send_request`.

### Test Case 6: Servidor - Procesador de Imágenes
**Archivo:** `server_test2_image_processor.rosblocks`

Servidor que simula el procesamiento de imágenes con diferentes filtros (desenfoque, nitidez, detección de bordes). Calcula tiempos de procesamiento y estadísticas sobre los píxeles afectados.

### Test Case 7: Cliente - Procesador de Imágenes
**Archivo:** `client_test_image_processor.rosblocks`

Cliente que solicita varios procesamientos de imágenes al servidor `ImageProcessor`. Prueba diferentes combinaciones de tamaños de imagen, tipos de filtros e intensidades, y reporta los resultados de rendimiento.

**Nota**: Utiliza el tipo correcto `ros_create_client` y `ros_send_request`.

### Test Case 8: Cliente - Secuencias Matemáticas
**Archivo:** `client_test1_math_sequence.rosblocks`

Cliente que solicita el cálculo de diferentes secuencias matemáticas (fibonacci, factorial, etc.) a un servidor. Prueba múltiples operaciones y mide el rendimiento de cada una.

**Nota**: Corregido para usar `ros_create_client` en lugar de `ros2_create_client`.

### Test Case 9: Cliente - Analizador de Texto
**Archivo:** `client_test2_text_analyzer.rosblocks`

Cliente que envía textos para ser analizados por un servidor. Realiza múltiples solicitudes para diferentes textos y tipos de análisis.

**Nota**: Corregido para usar `ros_create_client` en lugar de `ros2_create_client`.

## Caso Complejo (Planificado)
**Archivo:** `complex_service_test.rosblocks` (a implementar)

Este caso combinará múltiples servicios con clientes que intercambian información entre ellos, creando un flujo de datos más complejo.

## Guía de Uso

Para utilizar estos casos de prueba:

1. Cargar el archivo .rosblocks en el entorno ROSBLOCKS
2. Para archivos que solo contienen servidor o cliente, asegurar que el servicio correspondiente esté disponible
   - Los clientes `client_test_path_planner.rosblocks` y `client_test_image_processor.rosblocks` están diseñados para trabajar con sus servidores correspondientes: `server_test1_path_planner.rosblocks` y `server_test2_image_processor.rosblocks`
3. Ejecutar los nodos y observar los resultados en el terminal

## Notas Importantes

- **Nomenclatura de bloques**: Asegúrese de usar `ros_create_client` y `ros_create_server` (sin el "2"). Esto es crítico para que los archivos carguen correctamente.
- **Solicitudes de cliente**: Utilice `ros_send_request` para enviar solicitudes desde el cliente.
- Si encuentra problemas al cargar los archivos, verifique que está utilizando la versión correcta de ROSBLOCKS compatible con estos ejemplos.
- Estos casos de prueba están diseñados para ser ejemplos educativos y pueden requerir adaptaciones para entornos de producción. 