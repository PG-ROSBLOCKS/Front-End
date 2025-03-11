import * as Blockly from 'blockly/core';
import { pythonGenerator, Order, PythonGenerator } from 'blockly/python';
import { removeIndentation } from '../utilities/sanitizer-tools';
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
  ['Pose (turtlesim)', 'turtlesim.msg.Pose']
];
const common_msgs_for_custom: [string, string][] = [
  ['String (std_msgs)', 'string'],
  ['Bool (std_msgs)', 'bool'],
  ['Int64 (std_msgs)', 'int64'],
  ['Char (std_msgs)', 'char'],
  ['Float32 (std_msgs)', 'float32'], //TODO: Revisar que los tipos estén bien declarados para los .srv, ya que cambian cuando es un tipo de dato no primitivo
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
        .appendField("a");
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
          .appendField("Publicar Twist");
      this.appendValueInput("LINEAR")
          .setCheck("Number")
          .appendField("Velocidad Lineal:");
      this.appendValueInput("ANGULAR")
          .setCheck("Number")
          .appendField("Velocidad Angular:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Publica un mensaje Twist con velocidades lineal y angular definidas por el usuario y registra el mensaje en el log");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_rotate_turtle'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Rotar tortuga");
      this.appendValueInput("GRADOS")
          .setCheck("Number")
          .appendField("Grados:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Rota la tortuga un número de grados especificado.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_turtle_set_pose'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Posicionar tortuga");
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
      this.setColour(160);
      this.setTooltip("Posiciona la tortuga en una posición y orientación específicas.");
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
        .appendField("Servicio")
        .appendField(new Blockly.FieldTextInput("add_two_ints"), "SERVICE_NAME");
              this.appendDummyInput()
        .appendField("Timeout (segundos)")
        .appendField(new Blockly.FieldNumber(0.5, 0.1, 60, 0.1), "TIMER");
        this.appendDummyInput()
        .appendField("Mensaje base")
        .appendField(new Blockly.FieldTextInput("service not available, waiting again..."), "MESSAGE_BASE");

      // We add a “MAIN” space to connect a “child” block
      this.appendStatementInput("MAIN")
        .setCheck(null)
        .appendField("Procesar petición:");
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
  Blockly.Blocks['ros2_rotate_turtle'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Rotar tortuga");
      this.appendValueInput("GRADOS")
          .setCheck("Number")
          .appendField("Grados:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Rota la tortuga un número de grados especificado.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_turtle_set_pose'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Posicionar tortuga");
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
      this.setColour(160);
      this.setTooltip("Posiciona la tortuga en una posición y orientación específicas.");
      this.setHelpUrl("");
    }
  };
}

type ImportsDictionary = {
  [key: string]: Set<string>;
};

const importsDictMsgs: ImportsDictionary = {
  'std_msgs': new Set(),
  'geometry_msgs': new Set(),
  'nav_msgs': new Set(),
  'turtlesim': new Set()
};

const importsDictSrvs: ImportsDictionary = {
  'turtlesim': new Set()
};

function addImport(msgType: string): string {
  const msgParts = msgType.split('.msg.');
  const srvParts = msgType.split('.srv.');

  if (msgParts.length === 2) {
    const [packageName, msgName] = msgParts;
    if (importsDictMsgs[packageName]) {
      importsDictMsgs[packageName].add(msgName);
    }
    return msgName;
  } else if (srvParts.length === 2) {
    const [packageName, srvName] = srvParts;
    if (importsDictSrvs[packageName]) {
      importsDictSrvs[packageName].add(srvName);
    }
    return srvName;
  }

  return msgType;
}

function getImports(): string {
  let importCode = `import rclpy\nfrom rclpy.node import Node\n`;

  for (const [pkg, msgs] of Object.entries(importsDictMsgs)) {
      if (msgs.size > 0) {
          importCode += `from ${pkg}.msg import ${Array.from(msgs).join(', ')}\n`;
      }
  }

  for (const [pkg, srvs] of Object.entries(importsDictSrvs)) {
      if (srvs.size > 0) {
          importCode += `from ${pkg}.srv import ${Array.from(srvs).join(', ')}\n`;
      }
  }

  return importCode + '\n';
}

function clearImports(): void {
  for (const key in importsDictMsgs) {
      importsDictMsgs[key].clear();
  }
  for (const key in importsDictSrvs) {
      importsDictSrvs[key].clear();
  }
}

export { addImport, getImports, clearImports };

export function definirGeneradoresROS2() {
  // Code generator for the "Create publisher" block
  pythonGenerator.forBlock['ros2_create_publisher'] = function (block) {
    const topicName: string = block.getFieldValue('TOPIC_NAME');
    const msgType: string = block.getFieldValue('MSG_TYPE');

    const msgClass = addImport(msgType);

    const code = `pub_sub\n${TAB_SPACE}${TAB_SPACE}self.publisher_ = self.create_publisher(${msgClass}, '${topicName}', 10)\n`;
    return code;
  };

  pythonGenerator.forBlock['ros2_minimal_publisher'] = function (block) {
    const topic = block.getFieldValue('TOPIC_NAME');
    const msgType = block.getFieldValue('MSG_TYPE');
    const timer = block.getFieldValue('TIMER');
    const messageBase = block.getFieldValue('MESSAGE_BASE');
    const msgClass = addImport(msgType);

    let code = `${TAB_SPACE}${TAB_SPACE}self.publisher_ = self.create_publisher(${msgClass}, '${topic}', 10)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.timer_ = self.create_timer(${timer}, self.timer_callback)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.i = 0\n\n`;
    code += `${TAB_SPACE}def timer_callback(self):\n`;
    code += `${TAB_SPACE}${TAB_SPACE}msg = ${msgClass}()\n`;
    code += `${TAB_SPACE}${TAB_SPACE}msg.data = '${messageBase}: %d' % self.i\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.publisher_.publish(msg)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.get_logger().info('Publishing: "%s"' % msg.data)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.i += 1\n`;
    return code;
  };

  // Code generator for the "Create subscriber" block
  pythonGenerator.forBlock['ros2_create_subscriber'] = function (block) {
    const topic = block.getFieldValue('TOPIC_NAME');
    const msgType = block.getFieldValue('MSG_TYPE');
    // "Callback"
    const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');
    const msgClass = addImport(msgType);

    let code = `pub_sub\n${TAB_SPACE}${TAB_SPACE}self.subscription = self.create_subscription(${msgClass}, '${topic}', self.listener_callback, 10)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.subscription  # Evitar warning de variable no usada\n\n`;

    code += `${TAB_SPACE}def listener_callback(self, msg):\n`;

    // checks received type
    code += `${TAB_SPACE}${TAB_SPACE}if not isinstance(msg, ${msgClass}):\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}self.get_logger().error("Error: Tipo de mensaje incorrecto. Se esperaba ${msgClass}.")\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}return\n\n`;

    // try/except to catch excepcions
    code += `${TAB_SPACE}${TAB_SPACE}try:\n`;
    if (!callbackCode.trim()) {
      code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}pass\n`;
    } else {
      code += pythonGenerator.prefixLines(callbackCode, `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}`);
    }
    code += `${TAB_SPACE}${TAB_SPACE}except Exception as e:\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}self.get_logger().error("Error en el callback: {}".format(e))\n`;

    return code;
  };

  pythonGenerator.forBlock['ros2_subscriber_msg_data'] = function (block) {
    const code = 'msg.data';
    return [code, Order.ATOMIC];
  };

  pythonGenerator.forBlock['ros2_print_msg_type'] = function (block) {
    const code = `print("Tipo de dato recibido:", type(msg))\n`;
    return code;
  };

  // Code generator for the block "Publicar mensaje"
  pythonGenerator.forBlock['ros2_publish_message'] = function (block) {
    const topicName: string = block.getFieldValue('TOPIC_NAME');
    const msgType: string = block.getFieldValue('MSG_TYPE');
    const messageContent: string = block.getFieldValue('MESSAGE_CONTENT');

    const msgClass = addImport(msgType);

    let code = `${TAB_SPACE}${TAB_SPACE}msg = ${msgClass}()\n`;
    code += `${TAB_SPACE}${TAB_SPACE}msg.data = "${messageContent}"\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.publisher_.publish(msg)\n`;
    return code;
  };

  // Code generator for the block "Crear Timer"
  pythonGenerator.forBlock['ros2_timer'] = function (block) {
    const interval = block.getFieldValue('INTERVAL');
    const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');

    let code = `${TAB_SPACE}${TAB_SPACE}self.timer_ = self.create_timer(${interval}, self.timer_callback)\n`;
    code += `${TAB_SPACE}def timer_callback(self):\n`;
    code += pythonGenerator.prefixLines(removeIndentation(callbackCode), `${TAB_SPACE}${TAB_SPACE}`);
    return code;
  };

  // Code generator for the block "Log de ROS 2"
  pythonGenerator.forBlock['ros2_log'] = function (block) {
    const logLevel = block.getFieldValue('LOG_LEVEL');
    const message = pythonGenerator.valueToCode(block, 'MESSAGE', Order.NONE) || '""';

    const code = `${TAB_SPACE}${TAB_SPACE}${logLevel}(${message})\n`;
    return code;
  };

  // Python generator for the turtlesim msg field block
  pythonGenerator.forBlock['ros2_turtlesim_pose_field'] = function (block) {
    const field = block.getFieldValue('FIELD');
    const code = 'msg.' + field;
    return [code, Order.ATOMIC];
  };

  // Python generator for the service message block
  pythonGenerator.forBlock['ros2_named_message'] = function (block, generator) {
    var message_type = block.getFieldValue('MESSAGE_TYPE');
    var message_name = block.getFieldValue('MESSAGE_NAME');

    return `${message_type} ${message_name}\n`;
  };
  pythonGenerator.forBlock['ros2_service_block'] = function (block, generator) {
    var service_name = block.getFieldValue('SERVICE_NAME');
    var request_messages = generator.statementToCode(block, 'REQUEST_MESSAGES')
      .split('\n')
      .map(line => line.trim())  // Deletes spaces on each line
      .join('\n');

    var response_messages = generator.statementToCode(block, 'RESPONSE_MESSAGES')
      .split('\n')
      .map(line => line.trim())  // Deletes spaces on each line
      .join('\n');

    var code = `srv\n# Archivo ${service_name}.srv generado por ROSBlocks\n${request_messages}\n---\n${response_messages}`;
    return code;
  };

  pythonGenerator.forBlock['ros2_message_block'] = function (block, generator) {
    var message_name = block.getFieldValue('MESSAGE_NAME');
    var message_fields = generator.statementToCode(block, 'MESSAGE_FIELDS')
      .split('\n')
      .map(line => line.trim())  // Deletes spaces on each line
      .join('\n');

    var code = `msg\n# Archivo ${message_name}.msg generado por ROSBlocks\n${message_fields}`;
    return code;
  };

  pythonGenerator.forBlock['ros_create_server'] = function (block) {
    const serviceName = block.getFieldValue('SERVER_NAME');
    const serviceType = block.getFieldValue('SERVER_TYPE');
    let callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');
    // Deletes extra lines
    callbackCode = removeIndentation(callbackCode);

    // Usamos el nombre del servicio para crear la clase (puedes sanitizarlo según necesites)
    const nodeName = serviceName;

    let code = `server|${serviceType}\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.service_ = self.create_service(${serviceType}, '${serviceName}', self.service_callback)\n\n`;

    code += `${TAB_SPACE}def service_callback(self, request, response):\n`;
    code += `${TAB_SPACE}${TAB_SPACE}try:\n`;
    if (!callbackCode.trim()) {
      code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}pass\n`;
    } else {
      // Three identation levels
      code += pythonGenerator.prefixLines(callbackCode, `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}`);
      code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}return response\n`;
    }
    code += `${TAB_SPACE}${TAB_SPACE}except Exception as e:\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}self.get_logger().error("Error en el callback: {}".format(e))\n`;

    return code;
  };

  pythonGenerator.forBlock['ros2_publish_twist'] = function(block) {
    const msgClass = addImport('geometry_msgs.msg.Twist');
    let linear = pythonGenerator.valueToCode(block, 'LINEAR', Order.ATOMIC) || '0.0';
    let angular = pythonGenerator.valueToCode(block, 'ANGULAR', Order.ATOMIC) || '0.0';
    let code = `${TAB_SPACE}${TAB_SPACE}msg = ${msgClass}()\n`;
    code += `${TAB_SPACE}${TAB_SPACE}msg.linear.x = float(${linear})\n`;
    code += `${TAB_SPACE}${TAB_SPACE}msg.angular.z = float(${angular})\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.publisher_.publish(msg)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.get_logger().info("Mensaje Twist publicado")\n`;
    return code;
  };

  pythonGenerator.forBlock['ros2_rotate_turtle'] = function(block) {
    const degrees = pythonGenerator.valueToCode(block, 'GRADOS', Order.ATOMIC) || '0';
    
    const msgClass = addImport('geometry_msgs.msg.Twist');
  
    let code = `msg = ${msgClass}()\n`;
    code += `msg.angular.z = float(${degrees})\n`;
    code += `self.publisher_.publish(msg)\n`;
    code += `self.get_logger().info('Rotando tortuga ${degrees} grados.')\n`;
  
    return code;
  };

  pythonGenerator.forBlock['ros2_turtle_set_pose'] = function(block) {
    const x = pythonGenerator.valueToCode(block, 'X', Order.ATOMIC) || '0';
    const y = pythonGenerator.valueToCode(block, 'Y', Order.ATOMIC) || '0';
    const theta = pythonGenerator.valueToCode(block, 'THETA', Order.ATOMIC) || '0';
  
    const srvClass = addImport('turtlesim.srv.TeleportAbsolute');
  
    let code = `self.teleport_client = self.create_client(${srvClass}, 'turtle1/teleport_absolute')\n`;
    code += `req = ${srvClass}.Request()\n`;
    code += `req.x = float(${x})\n`;
    code += `req.y = float(${y})\n`;
    code += `req.theta = float(${theta})\n`;
    code += `self.teleport_client.call_async(req)\n`;
    code += `self.get_logger().info('Posicionando tortuga en (${x}, ${y}) con orientación ${theta}.')\n`;
  
    return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
  };

  pythonGenerator.forBlock['srv_variable'] = function (block) {
    // reads VAR_SECTION (already defined in the block when toolbox is created)
    const section = block.getFieldValue('VAR_SECTION');
    const varName = block.getFieldValue('VAR_NAME');
    // Goes back, for example, "request.a" o "response.sum"
    return [`${section}.${varName}`, Order.ATOMIC];
  };

  pythonGenerator.forBlock['srv_response_set_field'] = function (block) {
    const fieldCode = pythonGenerator.valueToCode(block, 'RESPONSE_FIELD', Order.NONE) || "response.campo";
    const valueCode = pythonGenerator.valueToCode(block, 'VALUE', Order.NONE) || "0";

    // Generates something like:  response.sum = request.a + request.b
    const code = `${fieldCode} = ${valueCode}\n`;
    return code;
  };

  pythonGenerator.forBlock['ros_create_client'] = function (block) {
    let clientType = block.getFieldValue('CLIENT_TYPE');
    const serviceName = block.getFieldValue('SERVICE_NAME');  // New field for service name
    const timer = block.getFieldValue('TIMER');
    const messageBase = block.getFieldValue('MESSAGE_BASE');
    // 1) Search on srvList the object SrvInfo needed and extract the fields of request
    const srvInfo = srvList.find(srv => srv.name === clientType);
    const requestFields = srvInfo?.variables.request || [];
    console.log(requestFields);

    clientType = clientType.replace('.srv', '');
    // 3) Generates class and construction
  // Start building the generated code; the first line is to identify the block type
  let code = `client|${clientType}\n`;
  
  // The next line is written inside the constructor (1 level of indentation)
  code += `${TAB_SPACE}${TAB_SPACE}self.cli = self.create_client(${clientType}, '${serviceName}')\n`;
  
  // The while is generated to wait for the service; the while line is at the same level as the previous one
  code += `${TAB_SPACE}${TAB_SPACE}while not self.cli.wait_for_service(timeout_sec=${timer}):\n`;
  // Inside the while, an additional indentation is added
  code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}self.get_logger().info('${messageBase}')\n`;
  
  // After the while, the request is created, at the same level as the while
  code += `${TAB_SPACE}${TAB_SPACE}self.req = ${clientType}.Request()\n\n`;
  
    // 4) Generate the "send_request" method with the request fields
    const paramList = requestFields.map(field => field.name).join(', ');
    code += `${TAB_SPACE}def send_request(self, ${paramList}):\n`;
  
    // 5) Assign each field in self.req.<field> = <field>
    requestFields.forEach(field => {
      code += `${TAB_SPACE}${TAB_SPACE}self.req.${field.name} = ${field.name}\n`;
    });
    // 6) Return asynchronous call
    code += `${TAB_SPACE}${TAB_SPACE}return self.cli.call_async(self.req)\n`;
  
    let main_code = pythonGenerator.statementToCode(block, 'MAIN');
    main_code = removeCommonIndentation(main_code);
    code += main_code;
    return code;
  };
  
  pythonGenerator.forBlock['ros2_service_available'] = function (block) {
    const timeout = block.getFieldValue('TIMEOUT') || 1.0;
    // Generate the code that returns True/False according to wait_for_service
    const code = `self.cli.wait_for_service(timeout_sec=${timeout})`;
    // The second element of the array is the order of precedence;
    // use pythonGenerator.ORDER_LOGICAL_NOT if you're combining it with a 'not'
    return [code, Order.NONE];
  };

  pythonGenerator.forBlock["ros_send_request"] = function (block) {
    const myBlock = block as any;  // ⬅ cast a 'any'
    const clientType = myBlock.clientType || "UnknownSrv";
    const requestFields = myBlock.requestFields || [];
    let values: any[] = [];

    // For each field => we generate lines “self.req.<field> = (whatever is in the input)”
    let assignments = "";
    for (const field of requestFields) {
      // Use the same identifier you used in the updateShape_
      const value = block.getFieldValue(field.name) || "0";
      values.push(`${field.name} = ${value}\n`);
    }

    let code = `#main-sendrequest\n`;
    // We assume that in the parent class we define “self.req = X.Request()” and “self.client_”
    code += assignments;

    //for each attribute of the request, send it to the method of the form a = input, b = input
    let request =  `future = node.send_request(${values.join(', ')})\n`;
    code += request;

    code += `rclpy.spin_until_future_complete(node, future)\n`;
    code += `response = future.result()\n`;
    code += `node.get_logger().info("Respuesta: {}".format(response))\n`;
    return code;
  };
}

/**
* Removes the minimum common indentation from all lines,
* preserving the difference between them (internal nesting).
*/
function removeCommonIndentation(code: string) {
  let lines = code.split('\n');

  // Remove blank lines at the beginning and end
  while (lines.length && !lines[0].trim()) {
    lines.shift();
  }
  while (lines.length && !lines[lines.length - 1].trim()) {
    lines.pop();
  }

  // Calculates minimum indentation
  let minIndent = Infinity;
  for (const line of lines) {
    if (!line.trim()) continue; // Ignore empty lines
    const match = line.match(/^(\s*)/);
    const indentCount = match ? match[1].length : 0;
    if (indentCount < minIndent) {
      minIndent = indentCount;
    }
  }
  if (minIndent === Infinity) {
    return code; // If there are no lines with content
  }

  // Deletes minIndent on every line
  lines = lines.map(line => line.slice(minIndent));
  return lines.join('\n');
}