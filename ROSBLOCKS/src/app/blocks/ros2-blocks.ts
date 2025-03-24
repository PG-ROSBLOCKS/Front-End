import * as Blockly from 'blockly/core';
import { srvList } from '../shared/srv-list';
import { msgList, MsgVariable } from '../shared/msg-list';
import { EventType } from 'blockly/core/events/type';
import { common_msgs, common_msgs_for_custom } from './ros2-msgs';

const TAB_SPACE = '    '; // Tab space
let srv_list = [];
export function definirBloquesROS2() {
  Blockly.Blocks['ros2_create_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('Create Publisher')
        .appendField(new Blockly.FieldTextInput('/my_topic'), 'TOPIC_NAME')
        .appendField('Type')
        .appendField(new Blockly.FieldDropdown(() => {
          const allOptionsMap = new Map<string, string>();
        
          // Agregar common_msgs (ya tienen su [label, value])
          common_msgs.forEach(([label, value]) => {
            allOptionsMap.set(value, label);
          });
        
          // Agregar mensajes personalizados sin modificar
          msgList.forEach((msg) => {
            const name = msg.name;
            // Si ya está en el mapa (por ejemplo, por common_msgs), lo ignoramos
            if (!allOptionsMap.has(name)) {
              allOptionsMap.set(name, name); // Mostrarlo tal cual
            }
          });
        
          const combinedOptions: [string, string][] = Array.from(allOptionsMap.entries()).map(
            ([value, label]) => [label, value]
          );
        
          return combinedOptions.length > 0 ? combinedOptions : [['Sin mensajes', '']];
        }, (newValue) => {
          this.messageType = newValue;
          this.updateChildren_();
          return newValue;
        }), 'MSG_TYPE');
      this.appendStatementInput('MAIN')
        .setCheck(null)
        .appendField('On publish');
  
      this.setColour(160);
      this.messageType = '';
  
      // ACTIVAMOS setOnChange con filtro
      // this.setOnChange((event: { type: EventType; blockId: any; element: string; name: string; }) => {
      //   // 1) Si es un BLOCK_CHANGE en ESTE bloque, revisamos si cambió MSG_TYPE
      //   if (event.type === Blockly.Events.BLOCK_CHANGE && event.blockId === this.id) {
      //     if (event.element === 'field' && event.name === 'MSG_TYPE') {
      //       // El usuario cambió el dropdown de tipo
      //       this.messageType = this.getFieldValue('MSG_TYPE');
      //       this.updateChildren_();
      //     }
      //   }
      //   // 2) O si es un BLOCK_MOVE, puede ser que estemos conectando un nuevo bloque
      //   //    a la sección 'MAIN' (o moviendo un timer/publish_message)
      //   else if (event.type === Blockly.Events.BLOCK_MOVE) {
      //     // No te preocupes en exceso por filtrar quién se movió.
      //     // Llamamos a updateChildren_ y que él haga la lógica de refrescar
      //     this.updateChildren_();
      //   }
      // });
    },
  
    mutationToDom: function () {
      const container = document.createElement('mutation');
      container.setAttribute('messageType', this.messageType || '');
      return container;
    },
  
    domToMutation: function (xmlElement: { getAttribute: (arg0: string) => string; }) {
      this.messageType = xmlElement.getAttribute('messageType') || '';
    },
  
    // Esta función recorre recursivamente un bloque y sus sub-bloques
    traverseAndUpdate_: function (block: { type: string; updateFromParent: (arg0: any) => void; inputList: { connection: { targetBlock: () => any; }; }[]; nextConnection: { targetBlock: () => any; }; }, messageType: any) {
      if (!block) return;
  
      // Si el bloque es ros2_timer o ros2_publish_message, actualizamos
      if (block.type === 'ros2_timer' || block.type === 'ros2_publish_message') {
        block.updateFromParent(messageType);
      }
  
      // Recorremos todos los inputs del bloque (tanto statement como value)
      block.inputList.forEach((input: { connection: { targetBlock: () => any; }; }) => {
        const target = input.connection && input.connection.targetBlock();
        if (target) {
          this.traverseAndUpdate_(target, messageType);
        }
      });
  
      // Y luego continuamos con el siguiente bloque en la cadena
      const next = block.nextConnection && block.nextConnection.targetBlock();
      if (next) {
        this.traverseAndUpdate_(next, messageType);
      }
    },
  
    updateChildren_: function() {
      const mainInput = this.getInput('MAIN');
      if (!mainInput || !mainInput.connection) return;
  
      // Obtenemos el primer bloque conectado a la entrada MAIN
      let currentBlock = mainInput.connection.targetBlock();
      // Recorremos todos los sub-bloques y next blocks
      this.traverseAndUpdate_(currentBlock, this.messageType);
    }
  };
  
  

  //MinimalPublisher (Just for testing)
  Blockly.Blocks['ros2_minimal_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Minimal Publisher")
        .appendField("Topic")
        .appendField(new Blockly.FieldTextInput("/my_topic"), "TOPIC_NAME")
        .appendField("Type")
        .appendField(new Blockly.FieldDropdown(common_msgs), "MSG_TYPE");
      this.appendDummyInput()
        .appendField("Timer (seconds)")
        .appendField(new Blockly.FieldNumber(0.5, 0.1, 60, 0.1), "TIMER");
      this.appendDummyInput()
        .appendField("Base Message")
        .appendField(new Blockly.FieldTextInput("Hello, ROS2!"), "MESSAGE_BASE");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Creates a minimal ROS2 publisher node");
      this.setHelpUrl("");
    }
  };

  // Block to create a subscriber
  Blockly.Blocks['ros2_create_subscriber'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Create Subscriber")
        .appendField(new Blockly.FieldTextInput("/mi_topico"), "TOPIC_NAME")
        .appendField("Type")
        .appendField(new Blockly.FieldDropdown(() => {
          const allOptionsMap = new Map<string, string>();
  
          // Añadir mensajes comunes
          common_msgs.forEach(([label, value]) => {
            allOptionsMap.set(value, label);
          });
  
          // Añadir mensajes del usuario si no están ya
          msgList.forEach((msg) => {
            if (!allOptionsMap.has(msg.name)) {
              allOptionsMap.set(msg.name, msg.name); // Mostrar tal cual
            }
          });
  
          return Array.from(allOptionsMap.entries()).map(([value, label]) => [label, value]);
        }, (newValue) => {
          this.messageType = newValue;
          return newValue;
        }), "MSG_TYPE");
  
      this.appendStatementInput("CALLBACK")
        .setCheck(null)
        .appendField("Callback");
  
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Creates a ROS2 subscriber node with a callback to process incoming messages.");
      this.setHelpUrl("");
  
      this.messageType = '';
    },
  
    mutationToDom: function () {
      const container = document.createElement('mutation');
      container.setAttribute('messageType', this.messageType || '');
      return container;
    },
  
    domToMutation: function (xmlElement: any) {
      this.messageType = xmlElement.getAttribute('messageType') || '';
    }
  };
  

  Blockly.Blocks['ros2_subscriber_msg_data'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Subscriber Response");
      // This block returns a value (output), so we use setOutput(true)
      this.setOutput(true, "String");
      this.setColour(230);
      this.setTooltip("Returns the content of msg.data (only valid inside listener_callback).");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_print_msg_type'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Print Received Data Type");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Prints the received message data type in the console.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['msg_variable'] = {
    init: function () {
      this.jsonInit({
        "type": "msg_variable",
        "message0": "%1 (%2)", 
        "args0": [
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
        "tooltip": "Campo definido en el mensaje (.msg)",
        "helpUrl": ""
      });
    }
  }; 

  // Block to publish a message
  Blockly.Blocks['ros2_publish_message'] = {
    init: function () {
      // Un input "dummy" solo para mostrar la etiqueta de tipo seleccionado
      this.appendDummyInput("TITLE")
        .appendField('Publish type:')
        .appendField(new Blockly.FieldLabelSerializable('Sin tipo'), 'MSG_TYPE');
  
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip('Publishes a message to a ROS 2 topic.');
      this.setHelpUrl('');
  
      // Variables internas
      this.messageType = '';
      this.messageFields = [];
      this.fieldValues = {};
    },
  
    /** Llamado por el bloque padre (ej. create_publisher) para asignar el tipo */
    updateFromParent: function (messageType: string) {
      this.messageType = messageType;
  
      // Actualizar etiqueta de tipo
      const label = this.getField('MSG_TYPE');
      if (label) {
        label.setValue(messageType.split('.').pop()?.replace(/\.msg$/, '') || messageType);
      }
  
      // Buscar en msgList la definición de campos
      const msgInfo = msgList.find(x => x.name === messageType);
      this.messageFields = msgInfo?.fields || [];
  
      this.updateShape_();
    },
  
    /** Reconstruye los inputs según la lista de messageFields */
    updateShape_: function () {
      // Guardar los valores actuales
      this.saveFieldValues();
  
      // Eliminar todos los inputs excepto TITLE
      const oldInputs = [...this.inputList];
      for (const input of oldInputs) {
        if (input.name !== "TITLE") {
          this.removeInput(input.name);
        }
      }
  
      // Añadir dinámicamente los campos
      this.addFieldsRecursively(this.messageFields, "");
    },
  
    /**
     * Método recursivo para anidar campos
     * Si el campo es otro mensaje, sube un nivel de recursión
     * Si no, crea un ValueInput con .setCheck(...) apropiado
     */
    addFieldsRecursively: function (fields: any, parentPath: any) {
      for (const field of fields) {
        const fullName = parentPath ? `${parentPath}.${field.name}` : field.name;
        const inputName = `FIELD_${fullName}`;
  
        // Determina si es un tipo de mensaje anidado
        const isNested = msgList.some(msg => msg.name === field.type || msg.name === `${field.type}.msg`);
  
        if (isNested) {
          // Recursión para subcampos
          const nested = msgList.find(m => m.name === field.type || m.name === `${field.type}.msg`);
          if (nested && nested.fields) {
            this.addFieldsRecursively(nested.fields, fullName);
          }
        } else {
          // Crear la "ranura" (hueco) para conectar un bloque
          const valueInput = this.appendValueInput(inputName)
            .appendField(fullName + ":");
  
          // Opcional: cargar un valor guardado (si no usas shadow blocks)
          // NOTA: Este "valor" no se mostrará directamente si es un ValueInput,
          //       porque la "edición" vendrá del bloque conectado.
          const saved = this.fieldValues[fullName] || "";
  
          // Ajusta .setCheck(...) según el tipo
          if (field.type === "string") {
            valueInput.setCheck("String");
  
            // // (Ejemplo) Si quieres un shadow block de texto por defecto:
            // const shadowBlock = Blockly.utils.xml.createElement('shadow');
            // shadowBlock.setAttribute('type', 'text');
            // const fieldNode = Blockly.utils.xml.createElement('field');
            // fieldNode.setAttribute('name', 'TEXT'); // para block "text"
            // fieldNode.textContent = saved; // valor por defecto
            // shadowBlock.appendChild(fieldNode);
            // valueInput.connection.setShadowDom(shadowBlock);
  
          } else if (["int64", "int32", "float64", "float32"].includes(field.type)) {
            valueInput.setCheck("Number");
  
            // // (Ejemplo) Shadow block numérico:
            // const shadowNumber = Blockly.utils.xml.createElement('shadow');
            // shadowNumber.setAttribute('type', 'math_number');
            // const fieldNum = Blockly.utils.xml.createElement('field');
            // fieldNum.setAttribute('name', 'NUM');
            // fieldNum.textContent = saved || "0";
            // shadowNumber.appendChild(fieldNum);
            // valueInput.connection.setShadowDom(shadowNumber);
  
          } else if (field.type === "bool") {
            valueInput.setCheck("Boolean");
  
            // // (Ejemplo) Shadow block boolean:
            // const shadowBool = Blockly.utils.xml.createElement('shadow');
            // shadowBool.setAttribute('type', 'logic_boolean');
            // valueInput.connection.setShadowDom(shadowBool);
  
          } else {
            // Tipo desconocido => permitir cualquier bloque
            valueInput.setCheck(null);
          }
        }
      }
    },
  
    /**
     * Guarda los valores de los inputs
     * (Solo sirve si estás usando .appendField() con FieldTextInput, etc.)
     * En caso de ValueInput, normalmente recuperas el valor en tu generador de código con `valueToCode`.
     */
    saveFieldValues: function () {
      for (const input of this.inputList) {
        if (input.name && input.name.startsWith("FIELD_")) {
          const fieldName = input.name.replace("FIELD_", "");
          const field = this.getField(fieldName);
          if (field) {
            this.fieldValues[fieldName] = field.getValue();
          }
        }
      }
    },
  
    /**
     * Mutations para serializar messageType, messageFields y fieldValues
     */
    mutationToDom: function () {
      const container = document.createElement('mutation');
      container.setAttribute('messageType', this.messageType || '');
      container.setAttribute('messageFields', JSON.stringify(this.messageFields));
      container.setAttribute('fieldValues', JSON.stringify(this.fieldValues));
      return container;
    },
  
    domToMutation: function (xmlElement: { getAttribute: (arg0: string) => string; }) {
      this.messageType = xmlElement.getAttribute('messageType') || '';
  
      try {
        this.messageFields = JSON.parse(xmlElement.getAttribute('messageFields') || "[]");
      } catch (e) {
        this.messageFields = [];
      }
  
      try {
        this.fieldValues = JSON.parse(xmlElement.getAttribute('fieldValues') || "{}");
      } catch (e) {
        this.fieldValues = {};
      }
  
      // Actualiza etiqueta si existe
      const label = this.getField('MSG_TYPE_LABEL');
      if (label) {
        label.setValue(
          this.messageType.split('.').pop()?.replace(/\.msg$/, '') || this.messageType
        );
      }
  
      this.updateShape_();
    }
  };
  

  // Block to create timer
  Blockly.Blocks['ros2_timer'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('Timer each')
        .appendField(new Blockly.FieldNumber(1, 0.1, Infinity, 0.1), 'INTERVAL')
        .appendField('seconds');
      this.appendStatementInput('CALLBACK')
        .appendField('Ejecutar');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(120);
      this.setTooltip('Creates a timer that executes periodically a callback.');
      this.setHelpUrl('');
      this.setInputsInline(false);
  
      this.messageType = ''; // para guardar el tipo de mensaje que viene del padre
  
      // setOnChange con filtro
      // this.setOnChange((event: { type: EventType; }) => {
      //   // Si este block no está conectado a nada, no hacemos nada
      //   if (!this.parentBlock_) return;
  
      //   // 1) Si es un BLOCK_CHANGE en este timer, mirar si cambió algo que requiera refrescar 
      //   //    (Por ahora, no hay un dropdown que indique tipo, pero podrías filtrar si 
      //   //    cambió un field que requiera refrescar.)
        
      //   // 2) O si es un BLOCK_MOVE => puede que conectemos un publish_message en 'CALLBACK'
      //   if (event.type === Blockly.Events.BLOCK_MOVE) {
      //     // Llamamos a updateChildren_ para refrescar a los publish_message conectados
      //     this.updateChildren_();
      //   }
      // });
    },
  
    updateFromParent: function(messageType: any) {
      this.messageType = messageType;
      // Cuando me avise mi padre, se lo paso a mis hijos
      this.updateChildren_();
    },
  
    updateChildren_: function() {
      const callbackInput = this.getInput('CALLBACK');
      if (!callbackInput?.connection) return;
  
      let childBlock = callbackInput.connection.targetBlock();
      while (childBlock) {
        if (childBlock.type === 'ros2_publish_message') {
          childBlock.updateFromParent(this.messageType);
        }
        // Siguiente en la cadena
        if (childBlock.nextConnection) {
          childBlock = childBlock.nextConnection.targetBlock();
        } else {
          childBlock = null;
        }
      }
    }
  };
  

  // Log ROS 2
  Blockly.Blocks['ros2_log'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("ROS2 Log level")
        .appendField(new Blockly.FieldDropdown([
          ["INFO", "self.get_logger().info"],
          ["DEBUG", "self.get_logger().debug"],
          ["WARNING", "self.get_logger().warning"],
          ["ERROR", "self.get_logger().error"],
          ["FATAL", "self.get_logger().fatal"]
        ]), "LOG_LEVEL");
      this.appendValueInput("MESSAGE")
        .setCheck("String")
        .appendField("Message");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Registers a menssage in the ROS2 log with the selected level.");
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
        .appendField("Type")
        .appendField(new Blockly.FieldDropdown(common_msgs_for_custom), "MESSAGE_TYPE")
        .appendField("Name")
        .appendField(new Blockly.FieldTextInput("parameter"), "MESSAGE_NAME");
      this.setPreviousStatement(true, "ros2_named_message");
      this.setNextStatement(true, "ros2_named_message");
      this.setColour(160);
      this.setTooltip("Defines a parameter for the .msg service");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_service_block'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Service")
        .appendField(new Blockly.FieldTextInput("MiServicio"), "SERVICE_NAME");
      this.appendStatementInput("REQUEST_MESSAGES") // Just accepts message blocks
        .setCheck("ros2_named_message")
        .appendField("Request");
      this.appendDummyInput()
        .appendField("---") // Separates between Request y Response en .srv
      this.appendStatementInput("RESPONSE_MESSAGES") // Just accepts message blocks
        .setCheck("ros2_named_message")
        .appendField("Response");
      this.setColour(230);
      this.setTooltip("Defines a custom service in ROS with parameters.");
      this.setHelpUrl("");
    }
  };
  Blockly.Blocks['ros2_message_block'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Defines Message")
        .appendField(new Blockly.FieldTextInput("MyMessage"), "MESSAGE_NAME");
      this.appendStatementInput("MESSAGE_FIELDS")
        .setCheck("ros2_named_message")
        .appendField("Message fields");
      this.setColour(230);
      this.setTooltip("Defines a custom messagefor ROS2.");
      this.setHelpUrl("");
    }
  };

  // On ros2-blocks.ts
  Blockly.Blocks['ros_create_server'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Create server")
        .appendField(new Blockly.FieldTextInput("MyServer"), "SERVER_NAME")
        .appendField("Type")
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
        .appendField("Response");
      this.setColour(230);
      this.setTooltip("Block to create a server in ROS2");
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
        "tooltip": "Variable defined in the service (.srv)",
        "helpUrl": ""
      });
    }
  };

  Blockly.Blocks['srv_response_set_field'] = {
    init: function () {
      // First input for the response variable, i.e. in a block that returns "response.sum"
      this.appendValueInput("RESPONSE_FIELD")
        .setCheck(null)
        .appendField("Assign");
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
      this.setTooltip("Assigns a value to a response object field in the callback.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_publish_twist'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Publish Twist from");
      this.appendDummyInput()
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.appendValueInput("LINEAR")
          .setCheck("Number")
          .appendField("Linear Velocity:");
      this.appendValueInput("ANGULAR")
          .setCheck("Number")
          .appendField("Angular Velocity:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Publishes a Twist message with user-defined linear and angular velocities and logs the message.");
      this.setHelpUrl("");
    }
  };

  //Block to create a client
  Blockly.Blocks["ros_create_client"] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Create Client")
        .appendField(new Blockly.FieldTextInput("minimal_client_async"), "CLIENT_NAME")
        .appendField("Type")
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
        .appendField("Server")
        .appendField(new Blockly.FieldTextInput("add_two_ints"), "SERVICE_NAME");
              this.appendDummyInput()
        .appendField("Server wait time ")
        .appendField(new Blockly.FieldNumber(0.5, 0.1, 60, 0.1), "TIMER")
        .appendField("(seconds)");
        this.appendDummyInput()
        .appendField("Server waiting message")
        .appendField(new Blockly.FieldTextInput("service not available, waiting again..."), "MESSAGE_BASE");

      // We add a “MAIN” space to connect a “child” block
      this.appendStatementInput("MAIN")
        .setCheck(null)
        .appendField("Process request");
      this.setColour(230);
      this.setTooltip("Block to create an asynchronous client in ROS2");
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
        .appendField("Service available (timeout:")
        .appendField(new Blockly.FieldNumber(1.0, 0.1, 60, 0.1), "TIMEOUT")
        .appendField("seg)");
      this.setOutput(true, "Boolean");
      this.setColour(210);
      this.setTooltip("Returns True if the service is available before the timeout, False otherwise.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks["ros_send_request"] = {
    init: function () {
      this.appendDummyInput("TITLE")
        .appendField("Send request to service");
      // Block type “statement” or “void”
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(230);
      this.setTooltip("Sends the request and waits for the response.");
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
          .appendField("Set turtle position");
      this.appendDummyInput()
          .appendField("Name:")
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
      this.setTooltip("Positions the turtle at a specific position and orientation.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_sleep'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Wait")
        .appendField(new Blockly.FieldNumber(1, 0.1, 60, 0.1), "SECONDS")
        .appendField("seconds");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Pauses execution for a specified number of seconds.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_kill_turtle'] = {
    init: function () {
      this.appendDummyInput()
          .appendField("Kill turtle:")
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Sends a request to kill a specified turtle.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_spawn_turtle'] = {
    init: function () {
      this.appendDummyInput()
          .appendField("Spawn turtle")
          .appendField("Name:")
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
      this.setTooltip("Creates a new turtle in turtlesim with name, position, and orientation.");
      this.setHelpUrl("");
    }
  };

  
  Blockly.Blocks['ros2_turtle_set_pen'] = {
    init: function () {
      this.appendDummyInput()
          .appendField("Change pen of")
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
          .appendField("Width:");
      
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
      this.setTooltip("Changes the color, width, and state of the turtle's pen in turtlesim.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_turtlesim_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('__init__')
      this.appendStatementInput('CALLBACK')
        .appendField('Execute');
      this.setPreviousStatement(false, null);
      this.setNextStatement(false, null);
      this.setColour(300);
      this.setTooltip('Creates a timer that periodically executes a callback.');
      this.setHelpUrl('');
    }
  };

  Blockly.Blocks['ros2_turtle_rotate'] = {
    init: function() {
      this.setInputsInline(true);
      this.appendDummyInput()
          .appendField("Rotate turtle");
      this.appendDummyInput()
          .appendField(new Blockly.FieldTextInput("turtle1"), "TURTLE_NAME");
      this.appendValueInput("GRADOS")
          .setCheck("Number")
          .appendField("Grados:");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(300);
      this.setTooltip("Rotates the turtle by a specified number of degrees.");
      this.setHelpUrl("");
    }
  };
}
