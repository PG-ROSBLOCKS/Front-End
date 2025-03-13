import * as Blockly from 'blockly/core';
import { srvList } from '../shared/srv-list';
import { EventType } from 'blockly/core/events/type';

const TAB_SPACE = '    '; // Tab space
let srv_list = [];
const common_msgs: [string, string][] = [
  ['String (std_msgs)', 'std_msgs.msg.String'],
  ['Bool (std_msgs)', 'std_msgs.msg.Bool'],
  ['Int64 (std_msgs)', 'std_msgs.msg.Int64'],
  ['Char (std_msgs)', 'std_msgs.msg.Char'],
  ['Float32 (std_msgs)', 'std_msgs.msg.Float32'],
  ['Twist (geometry_msgs)', 'geometry_msgs.msg.Twist'],
  ['Odometry (nav_msgs)', 'nav_msgs.msg.Odometry'],
  ['Pose (turtlesim)', 'turtlesim.msg.Pose'],

  ['Int16 (std_msgs)', 'std_msgs.msg.Int16'],
  ['Empty (std_msgs)', 'std_msgs.msg.Empty'],
  ['Float64 (std_msgs)', 'std_msgs.msg.Float64'],
  ['ColorRGBA (std_msgs)', 'std_msgs.msg.ColorRGBA'],
  ['Header (std_msgs)', 'std_msgs.msg.Header'],
  ['Byte (std_msgs)', 'std_msgs.msg.Byte'],

  //Geometry
  ['Pose (geometry_msgs)', 'geometry_msgs.msg.Pose'],
  ['Vector3 (geometry_msgs)', 'geometry_msgs.msg.Vector3'],
  ['Point (geometry_msgs)', 'geometry_msgs.msg.Point'],
  ['Quaternion (geometry_msgs)', 'geometry_msgs.msg.Quaternion'],
  ['Transform (geometry_msgs)', 'geometry_msgs.msg.Transform'],
  ['Pose2D (geometry_msgs)', 'geometry_msgs.msg.Pose2D'],
  ['Wrench (geometry_msgs)', 'geometry_msgs.msg.Wrench']
];
const common_msgs_for_custom: [string, string][] = [
  ['String (std_msgs)', 'string'],
  ['Bool (std_msgs)', 'bool'],
  ['Int64 (std_msgs)', 'int64'],
  ['Char (std_msgs)', 'char'],
  ['Float32 (std_msgs)', 'float32'],
  ['Twist (geometry_msgs)', 'geometry_msgs/Twist'],
  ['Odometry (nav_msgs)', 'nav_msgs/Odometry'],
  ['Pose (turtlesim)', 'turtlesim/Pose']
];

export function definirBloquesROS2() {
  Blockly.Blocks['ros2_create_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('Crear publicador')
        .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
        .appendField(new Blockly.FieldDropdown(common_msgs), 'MSG_TYPE');
      this.setNextStatement(true, null);
      this.setColour(160);
    }
  };

  //MinimalPublisher (Just for testing)
  Blockly.Blocks['ros2_minimal_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Minimal Publisher")
        .appendField("Tópico")
        .appendField(new Blockly.FieldTextInput("/mi_topico"), "TOPIC_NAME")
        .appendField("Tipo")
        .appendField(new Blockly.FieldDropdown(common_msgs), "MSG_TYPE");
      this.appendDummyInput()
        .appendField("Timer (segundos)")
        .appendField(new Blockly.FieldNumber(0.5, 0.1, 60, 0.1), "TIMER");
      this.appendDummyInput()
        .appendField("Mensaje base")
        .appendField(new Blockly.FieldTextInput("Hello, ROS2!"), "MESSAGE_BASE");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Crea un nodo minimal publisher de ROS2");
      this.setHelpUrl("");
    }
  };

  // Block to create a subscriber
  Blockly.Blocks['ros2_create_subscriber'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Crear suscriptor")
        .appendField(new Blockly.FieldTextInput("/mi_topico"), "TOPIC_NAME")
        .appendField(new Blockly.FieldDropdown(common_msgs), "MSG_TYPE");
      // C-shaped input
      this.appendStatementInput("CALLBACK")
        .setCheck(null)
        .appendField("Callback");
      // The previous connection is removed so that it cannot be joined by a higher block
      // this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Crea un nodo suscriptor de ROS2 con un callback para procesar mensajes entrantes.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_subscriber_msg_data'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("respuesta al suscriptor");
      // This block returns a value (output), so we use setOutput(true)
      this.setOutput(true, "String");
      this.setColour(230);
      this.setTooltip("Devuelve el contenido de msg.data (solo válido dentro de listener_callback).");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_print_msg_type'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Imprimir tipo de dato recibido");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Imprime en la consola el tipo de dato del mensaje recibido.");
      this.setHelpUrl("");
    }
  };

  // Block to publish a message
  Blockly.Blocks['ros2_publish_message'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('Publicar en')
        .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
        .appendField('tipo')
        .appendField(new Blockly.FieldDropdown(common_msgs), 'MSG_TYPE');
      this.appendDummyInput()
        .appendField('Mensaje')
        .appendField(new Blockly.FieldTextInput('Hola, ROS 2!'), 'MESSAGE_CONTENT');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip('Publica un mensaje en un tópico de ROS 2.');
      this.setHelpUrl('');
    }
  };

  // Block to create timer
  Blockly.Blocks['ros2_timer'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('Timer cada')
        .appendField(new Blockly.FieldNumber(1, 0.1, 60, 0.1), 'INTERVAL')
        .appendField('segundos');
      this.appendStatementInput('CALLBACK')
        .appendField('Ejecutar');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(120);
      this.setTooltip('Crea un timer que ejecuta un callback periódicamente.');
      this.setHelpUrl('');
    }
  };

  // Log ROS 2
  Blockly.Blocks['ros2_log'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("ROS2 Log nivel")
        .appendField(new Blockly.FieldDropdown([
          ["INFO", "self.get_logger().info"],
          ["DEBUG", "self.get_logger().debug"],
          ["WARNING", "self.get_logger().warning"],
          ["ERROR", "self.get_logger().error"],
          ["FATAL", "self.get_logger().fatal"]
        ]), "LOG_LEVEL");
      this.appendValueInput("MESSAGE")
        .setCheck("String")
        .appendField("Mensaje");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Registra un mensaje en el log de ROS 2 con el nivel seleccionado.");
      this.setHelpUrl("");
    }
  };

  // Block to retrieve a specific field from turtlesim.msg.Pose
  Blockly.Blocks['ros2_turtlesim_pose_field'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("turtlesim msg")
        .appendField(new Blockly.FieldDropdown([
          ["x", "x"],
          ["y", "y"],
          ["theta", "theta"],
          ["linear_velocity", "linear_velocity"],
          ["angular_velocity", "angular_velocity"]
        ]), "FIELD");
      this.setOutput(true, "Number"); // The output type is Number
      this.setColour(230);
      this.setTooltip("Returns the selected field from turtlesim.msg.Pose (only valid inside listener_callback).");
      this.setHelpUrl("");
    }
  };

  // Services
  Blockly.Blocks['ros2_named_message'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Tipo")
        .appendField(new Blockly.FieldDropdown(common_msgs_for_custom), "MESSAGE_TYPE")
        .appendField("Nombre")
        .appendField(new Blockly.FieldTextInput("parametro"), "MESSAGE_NAME");
      this.setPreviousStatement(true, "ros2_named_message");
      this.setNextStatement(true, "ros2_named_message");
      this.setColour(160);
      this.setTooltip("Define un parámetro para el servicio .srv");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_service_block'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Servicio")
        .appendField(new Blockly.FieldTextInput("MiServicio"), "SERVICE_NAME");
      this.appendStatementInput("REQUEST_MESSAGES") // Just accepts message blocks
        .setCheck("ros2_named_message")
        .appendField("Solicitud");
      this.appendDummyInput()
        .appendField("---") // Separates between Request y Response en .srv
      this.appendStatementInput("RESPONSE_MESSAGES") // Just accepts message blocks
        .setCheck("ros2_named_message")
        .appendField("Respuesta");
      this.setColour(230);
      this.setTooltip("Define un servicio personalizado en ROS con parámetros.");
      this.setHelpUrl("");
    }
  };
  Blockly.Blocks['ros2_message_block'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Definir Mensaje")
        .appendField(new Blockly.FieldTextInput("MiMensaje"), "MESSAGE_NAME");
      this.appendStatementInput("MESSAGE_FIELDS")
        .setCheck("ros2_named_message")
        .appendField("Campos del mensaje");
      this.setColour(230);
      this.setTooltip("Define un mensaje personalizado para ROS 2.");
      this.setHelpUrl("");
    }
  };

  // On ros2-blocks.ts
  Blockly.Blocks['ros_create_server'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Crear Servidor")
        .appendField(new Blockly.FieldTextInput("MiServidor"), "SERVER_NAME")
        .appendField("Tipo")
        // We use a function that returns the updated list with only the name (without extension)
        .appendField(new Blockly.FieldDropdown(() => {
          if (srvList.length > 0) {
            // For each service, we use its name without extension
            return srvList.map(srv => {
              // We remove the .srv extension if it is present
              const displayName = srv.name.replace(/\.srv$/, "");
              return [displayName, displayName];
            });
          } else {
            return [["Sin servicios", ""]];
          }
        }), "SERVER_TYPE");
      this.appendStatementInput("CALLBACK")
        .setCheck("Callback")
        .appendField("Respuesta");
      this.setColour(230);
      this.setTooltip("Bloque para crear un servidor en ROS2");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['srv_variable'] = {
    init: function () {
      this.jsonInit({
        "type": "srv_variable",
        "message0": "%1: %2 (%3)",   // Sample: section: name (type)
        "args0": [
          {
            "type": "field_label_serializable",
            "name": "VAR_SECTION",
            "text": "section"
          },
          {
            "type": "field_label_serializable",
            "name": "VAR_NAME",
            "text": "nombreVar"
          },
          {
            "type": "field_label_serializable",
            "name": "VAR_TYPE",
            "text": "tipoVar"
          }
        ],
        "output": null,
        "colour": 230,
        "tooltip": "Variable definida en el servicio (.srv)",
        "helpUrl": ""
      });
    }
  };

  Blockly.Blocks['srv_response_set_field'] = {
    init: function () {
      // First input for the response variable, i.e. in a block that returns "response.sum"
      this.appendValueInput("RESPONSE_FIELD")
        .setCheck(null)
        .appendField("Asignar");
      // Small tag for "a"
      this.appendDummyInput()
        .appendField("=");
      // Second input for the expression to be assigned
      this.appendValueInput("VALUE")
        .setCheck(null);
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Asigna un valor a un campo del objeto response en el callback.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_publish_twist'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Publicar Twist de");
      this.appendDummyInput()
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.appendValueInput("LINEAR")
          .setCheck("Number")
          .appendField("Velocidad Lineal:");
      this.appendValueInput("ANGULAR")
          .setCheck("Number")
          .appendField("Velocidad Angular:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Publica un mensaje Twist con velocidades lineal y angular definidas por el usuario y registra el mensaje en el log");
      this.setHelpUrl("");
    }
  };

  //Block to create a client
  Blockly.Blocks["ros_create_client"] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Crear Cliente")
        .appendField(new Blockly.FieldTextInput("minimal_client_async"), "CLIENT_NAME")
        .appendField("Tipo")
        .appendField(new Blockly.FieldDropdown(() => {
          // Options according to `srvList`
          if (srvList.length === 0) return [["Sin servicios", ""]];
          return srvList.map(srv => {
            const displayName = srv.name.replace(/\.srv$/, "");
            return [displayName, srv.name]; // Ej. ["AddTwoInts", "AddTwoInts.srv"]
          });
        }, (newValue) => {
          // When the dropdown changes, we save in the mutation
          this.clientType = newValue;
          // And we notify children (if they exist)
          this.updateChildren_();
          return newValue; // Ensure the validator returns a value
        }), "CLIENT_TYPE")
        .appendField("Servidor")
        .appendField(new Blockly.FieldTextInput("add_two_ints"), "SERVICE_NAME");
              this.appendDummyInput()
        .appendField("Tiempo de espera del servidor ")
        .appendField(new Blockly.FieldNumber(0.5, 0.1, 60, 0.1), "TIMER")
        .appendField("(segundos)");
        this.appendDummyInput()
        .appendField("Mensaje de espera del servidor")
        .appendField(new Blockly.FieldTextInput("service not available, waiting again..."), "MESSAGE_BASE");

      // We add a “MAIN” space to connect a “child” block
      this.appendStatementInput("MAIN")
        .setCheck(null)
        .appendField("Procesar petición");
      this.setColour(230);
      this.setTooltip("Bloque para crear un cliente asíncrono en ROS2");
      this.setHelpUrl("");
      // Internally we save the serviceType and the serviceName
      this.clientType = "";
      this.serviceName = this.getFieldValue("SERVICE_NAME");
      this.setOnChange((event: { type: EventType; recordUndo: any; }) => {
        // Check if the event is of type CREATE, CHANGE, MOVE, etc.
        // and if the user is not in the middle of an “undo/redo” (event.recordUndo).
        if (
          (event.type === Blockly.Events.BLOCK_CREATE ||
            event.type === Blockly.Events.BLOCK_MOVE ||
            event.type === Blockly.Events.BLOCK_CHANGE) && event.recordUndo
        ) {
          // Forces update of children
          this.updateChildren_();
        }
      });
    },
    // We save the info in the mutation
    mutationToDom: function () {
      const container = document.createElement('mutation');
      container.setAttribute('clientType', this.clientType || "");
      container.setAttribute('serviceName', this.getFieldValue("SERVICE_NAME") || "");
      return container;
    },
    domToMutation: function (xmlElement: { getAttribute: (arg0: string) => string; }) {
      this.clientType = xmlElement.getAttribute('clientType') || "";
      const serviceName = xmlElement.getAttribute('serviceName') || "";
      this.setFieldValue(serviceName, "SERVICE_NAME");
    },
    // Notify child blocks that the clientType has changed
    updateChildren_: function () {
      const mainInput = this.getInput("MAIN");
      if (mainInput && mainInput.connection && mainInput.connection.targetBlock()) {
        const childBlock = mainInput.connection.targetBlock();
        // If the child is “ros_send_request” (or whatever)
        if (childBlock && childBlock.type === "ros_send_request") {
          childBlock.updateFromParent(this.clientType);
        }
      }
    }
  };

  Blockly.Blocks['ros2_service_available'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("servicio disponible (timeout:")
        .appendField(new Blockly.FieldNumber(1.0, 0.1, 60, 0.1), "TIMEOUT")
        .appendField("seg)");
      this.setOutput(true, "Boolean");
      this.setColour(210);
      this.setTooltip("Devuelve True si el servicio está disponible antes del timeout, False si no.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks["ros_send_request"] = {
    init: function () {
      this.appendDummyInput("TITLE")
        .appendField("Enviar petición al servicio");
      // Block type “statement” or “void”
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Envía la petición y espera la respuesta.");
      this.setHelpUrl("");
      // Internal variables:
      this.clientType = "";       // Will be updated with “updateFromParent”
      this.requestFields = [];    // List of request fields
      this.fieldValues = {};      // *** Object to store the entered values
    },
    // Mutation
    mutationToDom: function () {
      const container = document.createElement('mutation');
      container.setAttribute('clientType', this.clientType);
      container.setAttribute('requestFields', JSON.stringify(this.requestFields));
      // *** Also save the values ​​in the mutation
      container.setAttribute('fieldValues', JSON.stringify(this.fieldValues));
      return container;
    },

    domToMutation: function (xmlElement: { getAttribute: (arg0: string) => string; }) {
      this.clientType = xmlElement.getAttribute('clientType') || "";
      try {
        this.requestFields = JSON.parse(xmlElement.getAttribute('requestFields') || "[]");
      } catch (e) {
        this.requestFields = [];
      }

      // *** Retrieve the values ​​saved in the mutation
      try {
        this.fieldValues = JSON.parse(xmlElement.getAttribute('fieldValues') || "{}");
      } catch (e) {
        this.fieldValues = {};
      }
      this.updateShape_();
    },

    /**
    * Called by the parent “ros_create_client” to assign the type
    * and redraw the inputs according to the fields in the request.
    */
    updateFromParent(newClientType: string) {
      this.clientType = newClientType;
      const info = srvList.find(x => x.name === newClientType);
      this.requestFields = info?.variables?.request || [];
      this.updateShape_();
    },

    updateShape_: function () {
      // 1) We save the current values ​​of the fields before redoing the shape
      this.saveFieldValues();
      // 2) We remove all inputs except “TITLE”
      const oldInputs = [...this.inputList];
      for (const inp of oldInputs) {
        if (inp.name !== "TITLE") {
          this.removeInput(inp.name);
        }
      }
      // 3) For each request field, we create a new DummyInput and assign the value that is saved
      for (const field of this.requestFields) {
        const inputName = "FIELD_" + field.name;
        const dummy = this.appendDummyInput(inputName);
        dummy.appendField(field.name + ": ");
        // Recuperamos el valor previo (si existe) o ponemos alguno por defecto
        const storedValue = this.fieldValues[field.name] || "";
        if (field.type === "int64") {
          // We recover the previous value (if it exists) or we put a default one
          const numValue = parseFloat(storedValue) || 0;
          dummy.appendField(new Blockly.FieldNumber(numValue), field.name);
        }
        else if (field.type === "string") {
          dummy.appendField(new Blockly.FieldTextInput(storedValue), field.name);
        }
        else if (field.type === "bool") {
          // If nothing is saved, we assume "True"
          // 1) Create an instance of FieldDropdown
          const boolField = new Blockly.FieldDropdown(
            [
              ["True", "True"],
              ["False", "False"]
            ]
          );
          // This value determines which initial option will be displayed
          boolField.setValue(storedValue === "False" ? "False" : "True");
          dummy.appendField(boolField, field.name);
        }
      }
    },
/**
* Read the current values ​​of the fields in the block and save them
* in this.fieldValues, to use them later and not lose the information.
*/
    saveFieldValues() {
      if (!this.requestFields) return;
      for (const f of this.requestFields) {
        const fieldObj = this.getField(f.name);
        if (fieldObj) {
          this.fieldValues[f.name] = fieldObj.getValue();
        }
      }
    }
  };
  
  Blockly.Blocks['ros2_turtle_set_pose'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Posicionar tortuga");
      this.appendDummyInput()
          .appendField("Nombre:")
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.appendValueInput("X")
          .setCheck("Number")
          .appendField("X:");
      this.appendValueInput("Y")
          .setCheck("Number")
          .appendField("Y:");
      this.appendValueInput("THETA")
          .setCheck("Number")
          .appendField("Theta:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Posiciona la tortuga en una posición y orientación específicas.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_sleep'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Esperar")
        .appendField(new Blockly.FieldNumber(1, 0.1, 60, 0.1), "SECONDS")
        .appendField("segundos");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Pausa la ejecución por un número determinado de segundos.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_kill_turtle'] = {
    init: function () {
      this.appendDummyInput()
          .appendField("Matar tortuga:")
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Envía solicitud para matar una tortuga especificada.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_spawn_turtle'] = {
    init: function () {
      this.appendDummyInput()
          .appendField("Spawnear tortuga")
          .appendField("Nombre:")
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
  
      this.appendValueInput("X")
          .setCheck("Number")
          .appendField("X:");
      this.appendValueInput("Y")
          .setCheck("Number")
          .appendField("Y:");
      this.appendValueInput("THETA")
          .setCheck("Number")
          .appendField("Theta:");
  
      this.setInputsInline(true);
  
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Crea una nueva tortuga en turtlesim con nombre, posición y orientación.");
      this.setHelpUrl("");
    }
  };

  
  Blockly.Blocks['ros2_turtle_set_pen'] = {
    init: function () {
      this.appendDummyInput()
          .appendField("Cambiar lápiz de")
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
  
      this.appendValueInput("R")
          .setCheck("Number")
          .appendField("Rojo:");
      this.appendValueInput("G")
          .setCheck("Number")
          .appendField("Verde:");
      this.appendValueInput("B")
          .setCheck("Number")
          .appendField("Azul:");
      this.appendValueInput("WIDTH")
          .setCheck("Number")
          .appendField("Grosor:");
      
      this.appendDummyInput()
          .appendField("Lápiz:")
          .appendField(new Blockly.FieldDropdown([
              ["Abajo", "0"],
              ["Arriba", "1"]
          ]), "PEN_STATE");
  
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Cambia el color, grosor y estado del lápiz de la tortuga en turtlesim.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_turtlesim_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('__init__')
      this.appendStatementInput('CALLBACK')
        .appendField('Ejecutar');
      this.setPreviousStatement(false, null);
      this.setNextStatement(false, null);
      this.setColour(300);
      this.setTooltip('Crea un timer que ejecuta un callback periódicamente.');
      this.setHelpUrl('');
    }
  };

  Blockly.Blocks['ros2_turtle_rotate'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Rotar tortuga");
      this.appendDummyInput()
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.appendValueInput("GRADOS")
          .setCheck("Number")
          .appendField("Grados:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Rota la tortuga un número de grados especificado.");
      this.setHelpUrl("");
    }
  };
}
