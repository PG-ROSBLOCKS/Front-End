# Casos de Prueba para Mensajes Personalizados en ROS2

Esta carpeta contiene casos de prueba diseñados para comprobar la compatibilidad y funcionalidad de los mensajes personalizados en ROS2 utilizando ROSBLOCKS.

## Lista de Casos de Prueba

| # | Nombre del Archivo | Descripción | Entradas | Precondiciones | Código de programación estructurada a probar en el bloque | Resultados Esperados |
|---|-------------------|-------------|----------|---------------|---------------------------------------------------|-------------------|
| 1 | custom_message_publisher.rosblocks | Caso básico con un mensaje personalizado (CustomPoint) y un publicador que envía datos 3D. | Coordenadas X, Y, Z | Ninguna | Creación y publicación de mensajes personalizados | Publicación correcta del mensaje. |
| 2 | custom_message_subscriber.rosblocks | Suscriptor que procesa datos de sensores (SensorData). | Datos de temperatura, humedad | Ninguna | Suscripción y procesamiento de mensajes personalizados | Procesamiento correcto de los datos recibidos. |
| 3 | nested_custom_message.rosblocks | Mensajes anidados con un mensaje base (Position2D) en un mensaje complejo (Robot). | Posición, orientación, nivel de batería | Ninguna | Creación y manipulación de mensajes anidados | Correcta creación y uso de mensajes anidados. |
| 4 | custom_service_with_custom_message.rosblocks | Servicio (ImageProcessor) utilizando un mensaje personalizado (ImageData). | Datos de imagen, parámetros de procesamiento | Ninguna | Definición y uso de servicios con mensajes personalizados | Procesamiento correcto de la solicitud y respuesta del servicio. |
| 5 | array_custom_message.rosblocks | Mensaje (PointCloud) conteniendo arrays para puntos 3D. | Conjunto de puntos 3D | Ninguna | Creación y manipulación de arrays en mensajes | Correcta gestión de arrays en mensajes. |
| 6 | multiple_publishers_same_message.rosblocks | Múltiples publicadores utilizando el mismo mensaje (StatusReport). | Estado de batería, estado de motor | Ninguna | Múltiples nodos publicando el mismo tipo de mensaje | Publicación correcta desde diferentes fuentes. |
| 7 | pub_sub_same_message.rosblocks | Publicador y suscriptor utilizando el mismo mensaje (RobotCommand). | Comandos de movimiento | Ninguna | Integración de publicador y suscriptor | Comunicación efectiva entre publicador y suscriptor. |
| 8 | service_with_message_dependency.rosblocks | Servicio (PathPlanner) dependiendo de un mensaje personalizado (Waypoint). | Puntos de inicio y fin, obstáculos | Ninguna | Servicio utilizando mensajes personalizados externos | Planificación correcta de ruta utilizando los datos proporcionados. |
| 9 | custom_message_inheritance.rosblocks | Herencia donde un mensaje (CarInfo) extiende un mensaje base (VehicleInfo). | Información del vehículo | Ninguna | Herencia y extensión de mensajes personalizados | Publicación correcta del mensaje. |
| 10 | mixed_service_message_complex.rosblocks | Sistema completo para procesamiento de datos de sensores con múltiples mensajes y servicios. | Lecturas de sensores, tipo de análisis | Ninguna | Integración compleja de múltiples mensajes y servicios | Flujo de datos funcionando correctamente desde el sensor hasta el análisis. |
| 11 | simple_publisher_test.rosblocks | Publicador simple con mensaje personalizado (SensorData) que demuestra interacciones con condicionales, ciclos, operaciones y variables. | ID del sensor, temperatura, humedad | Ninguna | Creación, manipulación y publicación de datos con estructuras de control | Publicación correcta del mensaje y manejo adecuado de alertas basadas en condiciones. |
| 12 | simple_subscriber_test1.rosblocks | Sistema simple que incluye un mensaje personalizado (RobotCommand), un suscriptor que procesa comandos y un publicador que genera comandos de movimiento. | Tipo de comando, velocidad lineal/angular, parada de emergencia | Ninguna | Creación, suscripción y publicación de comandos con procesamiento condicional | Publicación y procesamiento correcto de comandos con manejo adecuado de emergencias. |
| 13 | simple_subscriber_test2.rosblocks | Sistema de inventario con un mensaje personalizado (ProductStock), un suscriptor para monitoreo y un publicador que actualiza el inventario. | ID de producto, nombre, cantidad, precio | Ninguna | Publicación y suscripción de datos de inventario con cálculos y alertas | Actualizaciones de inventario correctas y alertas apropiadas para productos con bajo stock. |
| 14 | simple_pubsub_test1.rosblocks | Sistema pub-sub completo que maneja datos meteorológicos (WeatherData) con múltiples interacciones programáticas. | Ubicación, temperatura, humedad, presión, velocidad del viento, condición meteorológica | Ninguna | Integración de publicador y suscriptor con cálculos complejos y categorización de datos | Flujo completo de datos meteorológicos con alertas según condiciones y cálculo de índice de confort. |
| 15 | simple_pubsub_test2.rosblocks | Sistema pub-sub para seguimiento de entregas (DeliveryStatus) con ciclos, variables y múltiples operaciones de cálculo. | ID de paquete, estado, ubicación, horas estimadas, prioridad | Ninguna | Publicación secuencial de actualizaciones de entrega y procesamiento con análisis de prioridad | Generación correcta de actualizaciones de entrega y análisis apropiado de tiempos de entrega y prioridades. |

## Objetivos de los Casos de Prueba

Los casos de prueba están diseñados para verificar:

1. **Compatibilidad**: Asegurar que los mensajes personalizados funcionan correctamente en ROS2.
2. **Manejo de Mensajes Anidados**: Verificar que los mensajes pueden contener otros mensajes.
3. **Soporte de Arrays**: Comprobar que los mensajes pueden contener arrays de datos.
4. **Interoperabilidad**: Verificar que diferentes nodos pueden comunicarse utilizando el mismo tipo de mensaje personalizado.
5. **Dependencias entre Servicios y Mensajes**: Asegurar que los servicios pueden utilizar mensajes personalizados.
6. **Composición**: Verificar que se pueden crear sistemas complejos utilizando múltiples mensajes y servicios personalizados.
7. **Verificación de Eliminación en Cascada**: Comprobar que la eliminación de un mensaje utilizado por otros componentes no cause problemas.
8. **Estructuras de Control**: Verificar la interacción de mensajes personalizados con condicionales, ciclos, operaciones y variables.
9. **Sistemas Completos**: Probar la integración de publicadores y suscriptores en sistemas funcionales con lógica de procesamiento.

## Cómo Usar Estos Casos de Prueba

1. **Iniciar la Aplicación ROSBLOCKS**:
   - Abra la aplicación ROSBLOCKS.

2. **Cargar un Caso de Prueba**:
   - Vaya a "Archivo" > "Abrir".
   - Seleccione uno de los archivos .rosblocks de esta carpeta.

3. **Ver los Componentes Definidos**:
   - Explore las pestañas para ver los mensajes personalizados, publicadores, suscriptores y servicios definidos.
   - Examine cómo se estructuran los datos y cómo fluyen entre los componentes.

4. **Ejecutar el Caso de Prueba**:
   - Utilice los controles de ROSBLOCKS para iniciar los nodos y observar su comportamiento.
   - Verifique que los mensajes se publiquen correctamente y que los suscriptores procesen los datos según lo esperado.

5. **Verificar el Funcionamiento**:
   - Compare el comportamiento observado con los resultados esperados descritos en la tabla.
   - Utilice las herramientas de diagnóstico de ROSBLOCKS para identificar cualquier problema.

## Casos de Prueba Simples

Los casos de prueba 11-15 son especialmente útiles para principiantes, ya que demuestran patrones básicos de uso con mensajes personalizados sin la complejidad de los mensajes anidados. Estos casos han sido mejorados para incluir sistemas completos de publicador-suscriptor, asegurando que:

1. **Cada sistema es funcional por sí mismo**: Los casos incluyen tanto la definición del mensaje como los nodos de publicación y suscripción necesarios.

2. **Interacción con Estructuras de Control**: Demuestran el uso de condicionales, ciclos y operaciones matemáticas para procesar datos de los mensajes.

3. **Manipulación de Variables**: Muestran cómo extraer, transformar y utilizar datos de mensajes en variables locales.

4. **Toma de Decisiones Basada en Datos**: Implementan lógica que genera alertas y respuestas basadas en el contenido de los mensajes.

5. **Sistemas Pub-Sub Completos**: Proporcionan ejemplos de flujo de datos bidireccional con procesamiento en ambos extremos.

### Descripción detallada de los casos simples:

- **simple_publisher_test.rosblocks**: Un publicador que genera datos de sensores aleatorios y emite alertas cuando los valores superan ciertos umbrales.

- **simple_subscriber_test1.rosblocks**: Un sistema completo donde un publicador envía comandos de movimiento para un robot y un suscriptor los procesa, verificando condiciones de emergencia y calculando la magnitud total del movimiento.

- **simple_subscriber_test2.rosblocks**: Un sistema de inventario donde un publicador actualiza datos de productos y un suscriptor monitorea el stock, calculando valores totales y generando alertas para productos con bajo inventario.

- **simple_pubsub_test1.rosblocks**: Un sistema meteorológico completo donde un publicador genera datos climáticos para diferentes ubicaciones y un suscriptor los analiza, calculando índices de confort y generando alertas basadas en las condiciones.

- **simple_pubsub_test2.rosblocks**: Un sistema de seguimiento de entregas donde un publicador genera actualizaciones de estado de paquetes y un suscriptor los procesa, priorizando entregas y analizando plazos.

Estos casos de prueba simples son ideales para familiarizarse con el sistema de mensajería de ROS2 antes de pasar a escenarios más complejos. Se recomienda comenzar con estos ejemplos para comprender los conceptos básicos de comunicación entre nodos en ROS2. 