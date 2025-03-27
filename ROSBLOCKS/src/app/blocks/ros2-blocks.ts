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

      this.setOnChange((event: { type: EventType; blockId: any; element: string; name: string; }) => {
        if (!this.workspace || this.workspace.isDragging()) return;

        if (event.type === Blockly.Events.BLOCK_CHANGE && event.blockId === this.id) {
          if (event.element === 'field' && event.name === 'MSG_TYPE') {
            this.messageType = this.getFieldValue('MSG_TYPE');
            this.updateChildren_(); // aquí sí directo
          }
        } else if (event.type === Blockly.Events.BLOCK_MOVE) {
          // Delay para evitar conflictos durante el drag-and-drop
          setTimeout(() => {
            this.updateChildren_();
          }, 10); // puede ajustarse
        }
      });

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

    updateChildren_: function () {
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
        .appendField(new Blockly.FieldTextInput("/my_topic"), "TOPIC_NAME")
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
        .appendField("Topic Message Data");
      // This block returns a value (output), so we use setOutput(true)
      this.setOutput(true, "String");
      this.setColour(160);
      this.setTooltip("Returns the content of msg.data (only valid inside listener_callback).");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_print_msg_type'] = {
    init: function () {
      this.appendValueInput("VAR_EXPR")
        .setCheck(null)
        .appendField("Print type of");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Prints the type of the selected variable or expression.");
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
      // A "dummy" input just to display the selected type label
      this.appendDummyInput("TITLE")
        .appendField('Publish type:')
        .appendField(new Blockly.FieldLabelSerializable('No type'), 'MSG_TYPE');

      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip('Publishes a message to a ROS 2 topic.');
      this.setHelpUrl('');

      // Internal variables
      this.messageType = '';
      this.messageFields = [];
      this.fieldValues = {};

      // Add warning when the block is not dynamic (does not accept drag and drop of other blocks)
      this.setOnChange((event: any) => {
        // Si el workspace no existe o el usuario está arrastrando un bloque, no hacemos nada
        if (!this.workspace || this.workspace.isDragging()) return;

        let warningMessage = null;  // Variable para almacenar el mensaje final

        // 1. Verificar si el bloque es dinámico (si tiene messageType)
        const isDynamic = !!this.messageType;
        if (!isDynamic) {
          // Caso no dinámico => mostrar advertencia
          warningMessage = "This block must be inside a 'Create Publisher'.";

          // Si tiene inputs dinámicos, los limpiamos
          const hasDynamicInputs = this.inputList.some(
            (input: { name: string; }) => input.name?.startsWith("FIELD_")
          );
          if (hasDynamicInputs) {
            this.messageFields = [];
            this.updateShape_(); // Quita los inputs del bloque
          }
        } else {
          // 2. Si es dinámico, verificar si hay campos obligatorios sin conectar
          const hasUnconnectedFields = this.inputList.some((input: { name: string; connection: { targetBlock: () => any; }; }) =>
            input.name?.startsWith("FIELD_") &&
            !input.connection?.targetBlock()
          );

          if (hasUnconnectedFields) {
            warningMessage = 'You must complete all required fields before executing.';
          }
        }

        // 3. Finalmente, establecer (o limpiar) el warning solo UNA vez
        this.setWarningText(warningMessage);
      });


    },

    /** Called by the parent block (e.g., create_publisher) to assign the type */
    updateFromParent: function (messageType: string) {
      const alreadyConnected = this.inputList.some((input: { name: string; connection: { targetBlock: () => any; }; }) =>
        input.name?.startsWith("FIELD_") && input.connection?.targetBlock()
      );

      // Only regenerate if there are no connected fields
      if (this.messageType !== messageType && !alreadyConnected) {
        this.messageType = messageType;

        const label = this.getField('MSG_TYPE');
        if (label) {
          label.setValue(messageType.split('.').pop()?.replace(/\.msg$/, '') || messageType);
        }

        const msgInfo = msgList.find(x => x.name === messageType);
        this.messageFields = msgInfo?.fields || [];
        this.updateShape_();
      }
    }
    ,

    /** Rebuilds the inputs according to the messageFields list */
    updateShape_: function () {
      // Save current values
      this.saveFieldValues();

      // Remove all inputs except TITLE
      const oldInputs = [...this.inputList];
      for (const input of oldInputs) {
        if (input.name !== "TITLE") {
          this.removeInput(input.name);
        }
      }

      // Dynamically add fields
      this.addFieldsRecursively(this.messageFields, "");
    },

    /**
     * Recursive method to nest fields
     * If the field is another message, go up one level of recursion
     * Otherwise, create a ValueInput with .setCheck(...) as appropriate
     */
    addFieldsRecursively: function (fields: any, parentPath: any) {
      for (const field of fields) {
        const fullName = parentPath ? `${parentPath}.${field.name}` : field.name;
        const inputName = `FIELD_${fullName}`;

        // Determine if it is a nested message type
        const isNested = msgList.some(msg => msg.name === field.type || msg.name === `${field.type}.msg`);

        if (isNested) {
          // Recursion for subfields
          const nested = msgList.find(m => m.name === field.type || m.name === `${field.type}.msg`);
          if (nested && nested.fields) {
            this.addFieldsRecursively(nested.fields, fullName);
          }
        } else {
          // Create the "slot" (hole) to connect a block
          const valueInput = this.appendValueInput(inputName)
            .appendField(fullName + ":");

          // Optionally: load a saved value (if not using shadow blocks)
          // NOTE: This "value" will not be displayed directly if it is a ValueInput,
          //       because the "editing" will come from the connected block.
          const saved = this.fieldValues[fullName] || "";

          // Adjust .setCheck(...) according to the type
          if (field.type === "string") {
            valueInput.setCheck("String");

          } else if (["int64", "int32", "float64", "float32"].includes(field.type)) {
            valueInput.setCheck("Number");

          } else if (field.type === "bool") {
            valueInput.setCheck("Boolean");

          } else {
            // Unknown type => allow any block
            valueInput.setCheck(null);
          }
        }
      }
    },

    /**
     * Save the values of the inputs
     * (Only useful if you are using .appendField() with FieldTextInput, etc.)
     * In the case of ValueInput, you normally retrieve the value in your code generator with `valueToCode`.
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
     * Mutations to serialize messageType, messageFields, and fieldValues
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

      // Update label if it exists
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
        .appendField('Timer every')
        .appendField(new Blockly.FieldNumber(1, 0.1, Infinity, 0.1), 'INTERVAL')
        .appendField('seconds');
      this.appendStatementInput('CALLBACK')
        .appendField('Execute');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(120);
      this.setTooltip('Creates a timer that periodically executes a callback.');
      this.setHelpUrl('');
      this.setInputsInline(false);

      this.messageType = ''; // to store the message type coming from the parent

      this.setOnChange((event: { type: EventType; blockId: any; element: string; name: string; }) => {
        if (!this.workspace || this.workspace.isDragging()) return;

        if (event.type === Blockly.Events.BLOCK_CHANGE && event.blockId === this.id) {
          if (event.element === 'field' && event.name === 'MSG_TYPE') {
            this.messageType = this.getFieldValue('MSG_TYPE');
            this.updateChildren_(); // directly update here
          }
        } else if (event.type === Blockly.Events.BLOCK_MOVE) {
          // Delay to avoid conflicts during drag-and-drop
          setTimeout(() => {
            this.updateChildren_();
          }, 10); // can be adjusted
        }
      });

    },

    updateFromParent: function (messageType: any) {
      this.messageType = messageType;
      // When notified by the parent, pass it to the children
      this.updateChildren_();
    },

    updateChildren_: function () {
      const callbackInput = this.getInput('CALLBACK');
      if (!callbackInput?.connection) return;

      let childBlock = callbackInput.connection.targetBlock();
      while (childBlock) {
        if (childBlock.type === 'ros2_publish_message') {
          childBlock.updateFromParent(this.messageType);
        }
        // Next in the chain
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
        .appendField(new Blockly.FieldTextInput("MyService"), "SERVICE_NAME");
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
            return [["No services", ""]];
          }
        }), "SERVER_TYPE");
      this.appendStatementInput("CALLBACK")
        .setCheck("Callback")
        .appendField("Response");
      this.setColour(230);
      this.setTooltip("Block to create a server in ROS2");
      this.setHelpUrl("");

      this.setOnChange((event: { type: EventType; }) => {
        if (!this.workspace || this.workspace.isDragging()) return;

        if (event.type === Blockly.Events.BLOCK_MOVE) {
          setTimeout(() => {
            const selectedService = this.getFieldValue("SERVER_TYPE") || "";
            const selectedServiceNormalized = selectedService.replace(/\.srv$/, "");

            const callbackInput = this.getInput("CALLBACK");
            if (callbackInput && callbackInput.connection && callbackInput.connection.targetBlock()) {
              const childBlock = callbackInput.connection.targetBlock();
              validateDescendants(childBlock, selectedServiceNormalized);
            }
          }, 10);
        }
      });
    },
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
  init: function () {
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
      .appendField("Service Type")
      .appendField(new Blockly.FieldDropdown(() => {
        // Options according to `srvList`
        if (srvList.length === 0) return [["No services", ""]];
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
      .appendField(new Blockly.FieldTextInput("MyServer"), "SERVICE_NAME");
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
    this.setOnChange((event: { type: EventType; blockId: any; element: string; name: string; }) => {
      if (!this.workspace || this.workspace.isDragging()) return;

      if (event.type === Blockly.Events.BLOCK_CHANGE && event.blockId === this.id) {
        if (event.element === 'field' && event.name === 'MSG_TYPE') {
          this.messageType = this.getFieldValue('MSG_TYPE');
          this.updateChildren_(); // aquí sí directo
        }
      } else if (event.type === Blockly.Events.BLOCK_MOVE) {
        // Delay para evitar conflictos durante el drag-and-drop
        setTimeout(() => {
          this.updateChildren_();
          const selectedService = this.getFieldValue("CLIENT_TYPE") || "";
          const selectedServiceNormalized = selectedService.replace(/\.srv$/, "");

          // Verifica el árbol completo a partir del input "MAIN"
          const mainInput = this.getInput("MAIN");
          if (mainInput && mainInput.connection && mainInput.connection.targetBlock()) {
            const childBlock = mainInput.connection.targetBlock();
            validateDescendants(childBlock, selectedServiceNormalized);
          }
        }, 10); // puede ajustarse
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

function validateDescendants(block: { type: string; data: string; unplug: () => void; inputList: any[]; nextConnection: { targetBlock: () => any; }; }, selectedServiceNormalized: string) {
  if (!block) return;

  // Validar si el bloque es del tipo "srv_variable"
  if (block.type === "srv_variable") {
    const blockService = block.data || "";
    const blockServiceNormalized = blockService.replace(/\.srv$/, "");
    if (blockService && blockServiceNormalized !== selectedServiceNormalized) {
      alert(
        "El bloque de variable de servicio (" +
        blockServiceNormalized +
        ") no coincide con el servicio seleccionado (" +
        selectedServiceNormalized +
        ")"
      );
      block.unplug();
      // Opcional: Puedes retornar aquí si deseas detener la validación en esta rama
      // return;
    }
  }

  // Recorrer todas las conexiones de entrada (inputs)
  block.inputList.forEach((input) => {
    if (input.connection && input.connection.targetBlock()) {
      validateDescendants(input.connection.targetBlock(), selectedServiceNormalized);
    }
  });

  // Además, verificar la conexión "next" (para bloques conectados en secuencia)
  if (block.nextConnection && block.nextConnection.targetBlock()) {
    validateDescendants(block.nextConnection.targetBlock(), selectedServiceNormalized);
  }
}

Blockly.Blocks["ros_send_request"] = {
  init: function () {
    this.appendDummyInput("TITLE")
      .appendField("Send request to service");
    // Block type “statement” or “void”
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("Send the request and wait for the response.");
    this.setHelpUrl("");
    // Internal variables:
    this.clientType = "";       // Will be updated with “updateFromParent”
    this.requestFields = [];    // List of request fields
    this.fieldValues = {};      // *** Object to store the entered values
    this.setOnChange((event: any) => {
      if (!this.workspace || this.workspace.isDragging()) return;

      const parent = this.getSurroundParent();
      if (!parent || parent.type !== 'ros_create_client') return;

      let warningMessage = null;  // Variable to store the warning message

      const mainInput = parent.getInput('MAIN');
      const firstBlock = mainInput?.connection?.targetBlock();
      const isFirst = firstBlock && firstBlock.id === this.id;

      if (!isFirst) {
        // If it's not the first block, reset clientType and clear inputs
        if (this.clientType !== '') {
          this.clientType = '';
          this.requestFields = [];
          this.updateShape_();
        }
        warningMessage = 'This block can only be in the first position of the client.';
      } else {
        // Check if there is at least one unconnected slot
        const hasUnconnectedFields = this.inputList.some((input: { name: string; connection: { targetBlock: () => any; }; }) =>
          input.name?.startsWith("FIELD_") &&
          !input.connection?.targetBlock()
        );
        if (hasUnconnectedFields) {
          warningMessage = 'You must complete all required fields before executing.';
        }
      }

      // Set (or clear) the warning only once
      this.setWarningText(warningMessage);
    });


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
    const alreadyConnected = this.inputList.some((input: { name: string; connection: { targetBlock: () => any; }; }) =>
      input.name?.startsWith("FIELD_") && input.connection?.targetBlock()
    );

    // Solo regenerar si el tipo cambia Y no hay campos conectados
    if (this.clientType !== newClientType && !alreadyConnected) {
      this.clientType = newClientType;

      const info = srvList.find(x => x.name === newClientType);
      this.requestFields = info?.variables?.request || [];

      this.updateShape_();
    }
  },

  updateShape_: function () {
    this.saveFieldValues();

    // Eliminar todos los inputs excepto TITLE
    const oldInputs = [...this.inputList];
    for (const inp of oldInputs) {
      if (inp.name !== "TITLE") {
        this.removeInput(inp.name);
      }
    }

    // Crear ranuras tipo ValueInput para cada campo del request
    for (const field of this.requestFields) {
      const inputName = `FIELD_${field.name}`;
      const valueInput = this.appendValueInput(inputName)
        .appendField(field.name + ":");

      // Opcionalmente puedes guardar valores default si no hay bloque conectado
      const saved = this.fieldValues[field.name] || "";

      // Ajusta el tipo de dato esperado (setCheck) según el tipo del campo
      if (field.type === "string") {
        valueInput.setCheck("String");
      } else if (["int64", "int32", "float64", "float32"].includes(field.type)) {
        valueInput.setCheck("Number");
      } else if (field.type === "bool") {
        valueInput.setCheck("Boolean");
      } else {
        valueInput.setCheck(null); // Tipo desconocido => permitir cualquier bloque
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
      const input = this.getInput(`FIELD_${f.name}`);
      const connectedBlock = input?.connection?.targetBlock();

      // Si está conectado, no se guarda valor (se usará desde el bloque conectado)
      if (!connectedBlock) {
        const field = this.getField(f.name); // Por compatibilidad, si existe
        if (field) {
          this.fieldValues[f.name] = field.getValue();
        }
      }
    }
  }
};


Blockly.Blocks['ros2_turtle_set_pose'] = {
  init: function () {
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
    this.setTooltip("Send a request to kill a specified turtle.");
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
  init: function () {
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
// BLOCKS FOR COMMON TYPES
Blockly.Blocks['text_char_to_ascii'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("ASCII of")
      .appendField(new Blockly.FieldTextInput("a", this.validateChar), "CHAR");
    this.setOutput(true, "Number");
    this.setColour(160);
    this.setTooltip("Returns the ASCII code of a single character");
    this.setHelpUrl("");
  },

  validateChar: function (text: string): string {
    return text.length === 1 ? text : text.charAt(0); // Enforces only 1 character
  }
};
Blockly.Blocks['text_ascii_to_char'] = {
  init: function () {
    this.appendValueInput("ASCII_CODE")
      .appendField("Char from ASCII code");

    this.setOutput(true, "String");
    this.setColour(160);
    this.setTooltip("Converts an ASCII number to a character.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['float_number'] = {
  init: function () {
    this.appendDummyInput()
      .appendField(new Blockly.FieldTextInput("0.0", this.validateFloat), "NUM");
    this.setOutput(true, "Number");
    this.setColour(230);
    this.setTooltip("Número flotante explícito con precisión de hasta float64 (17 cifras significativas)");
    this.setHelpUrl("");
  },

  // Validate only explicit floats (requires decimal point or scientific notation)
  validateFloat: function (text: string) {
    // We only accept numbers with a decimal point or scientific notation
    const floatRegex = /^-?\d+\.\d+([eE][-+]?\d+)?$/;
    if (!floatRegex.test(text)) return null;

    const num = parseFloat(text);
    if (!isFinite(num)) return null;

    // Limit to float64 precision (17 significant digits)
    const matchResult = num.toExponential().replace('.', '').match(/\d+(?=e)/);
    const significantDigits = matchResult ? matchResult[0].length : 0;
    if (significantDigits > 17) return null;

    return text; // valid, it is preserved as is
  }
};

