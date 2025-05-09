import * as Blockly from 'blockly/core';
import { srvList } from '../shared/srv-list';
import { customMsgList, msgList, MsgVariable } from '../shared/msg-list';
import { EventType } from 'blockly/core/events/type';
import { common_msgs, common_msgs_for_custom } from './ros2-msgs';
import type { MessageService } from '../shared/message.service'; // asegúrate del path correcto
import { blockColors } from './color-palette';
import { sanitizeBaseNameAllowUnderscoreWithoutExtension, sanitizeNameWithoutExtension, validateTopicName } from '../utilities/sanitizer-tools';

let messageServiceInstance: MessageService | null = null;

export function setMessageService(service: MessageService) {
  messageServiceInstance = service;
}


const TAB_SPACE = '    '; // Tab space
let srv_list = [];
export function definirBloquesROS2() {
  Blockly.Blocks['ros2_create_publisher'] = {
    init: function () {
      this.appendDummyInput()
        .appendField('Create Publisher')
        .appendField(new Blockly.FieldTextInput('/my_topic', validateTopicName), 'TOPIC_NAME')
        .appendField('Type')
        .appendField(new Blockly.FieldDropdown(() => {
          const allOptionsMap = new Map<string, string>();

          // Agregar common_msgs (ya tienen su [label, value])
          common_msgs.forEach(([label, value]) => {
            allOptionsMap.set(value, label);
          });

          // Agregar mensajes personalizados sin modificar
          customMsgList.forEach((msg) => {
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
      this.setColour(blockColors.Topics);

      this.messageType = '';

      this.setOnChange((event: { type: EventType; blockId: any; element: string; name: string; }) => {
        const topic = this.getFieldValue('TOPIC_NAME') || '';
        if (topic === '/') {
            this.setWarningText('The topic name cannot be empty.');
        } else {
          this.setWarningText(null);
        }
        if (!this.workspace || this.workspace.isDragging()) return;
        if (event.type === Blockly.Events.BLOCK_CHANGE && event.blockId === this.id) {
          if (event.element === 'field' && event.name === 'MSG_TYPE') {
            this.messageType = this.getFieldValue('MSG_TYPE');
            this.updateChildren_(); 
          }
        } else if (event.type === Blockly.Events.BLOCK_MOVE) {
          setTimeout(() => {
            this.updateChildren_();
          }, 10); 
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
      const label = this.getField('MSG_TYPE');
      if (label) {
        label.setValue(
          this.messageType.split('.').pop()?.replace(/\.msg$/, '') || this.messageType
        );
      }

      // Now that updateShape_ preserves connections, call it here to build the block shape.
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
        .appendField(new Blockly.FieldTextInput("/my_topic", validateTopicName), "TOPIC_NAME")
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
        .appendField(new Blockly.FieldTextInput("/my_topic", validateTopicName), "TOPIC_NAME")
        .appendField("Type")
        .appendField(new Blockly.FieldDropdown(() => {
          const allOptionsMap = new Map<string, string>();

          // Añadir mensajes comunes
          common_msgs.forEach(([label, value]) => {
            allOptionsMap.set(value, label);
          });

          // Añadir mensajes del usuario si no están ya
          customMsgList.forEach((msg) => {
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
      this.setColour(blockColors.Topics);
      this.setTooltip("Creates a ROS2 subscriber node with a callback to process incoming messages.");
      this.setHelpUrl("");

      this.messageType = '';

      this.setOnChange((event: { type: EventType; blockId: any; element: string; name: string; }) => {
        const topic = this.getFieldValue('TOPIC_NAME') || '';
        if (topic === '/') {
            this.setWarningText('The topic name cannot be empty.');
        } else {
          this.setWarningText(null);
        }
      }
      );
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
      this.setColour(blockColors.Topics);
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
      this.setColour(blockColors.Topics);
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
        .appendField(new Blockly.FieldLabelSerializable('Select type'), 'MSG_TYPE');

      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(blockColors.Topics);
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

        const msgInfo = customMsgList.find(x => x.name === messageType);
        this.messageFields = msgInfo?.fields || [];
        this.updateShape_();
      }
    }
    ,

    /** Rebuilds the inputs according to the messageFields list */
    updateShape_: function () {
      //console.log(`[${this.id}] updateShape_ START for type: ${this.messageType}`);
      // 1. Save existing connections
      const savedConnections: { [inputName: string]: Blockly.Connection | null } = {};
      for (const input of this.inputList) {
        if (input.name && input.name.startsWith("FIELD_") && input.connection && input.connection.targetConnection) {
          savedConnections[input.name] = input.connection.targetConnection;
          console.log(`[${this.id}] updateShape_ - Saved connection for ${input.name}:`, input.connection.targetConnection);
        }
      }

      // Save current field values (if any are direct fields, not value inputs)
      this.saveFieldValues();

      // 2. Remove previous dynamic inputs (except TITLE)
      const oldInputs = [...this.inputList];
      for (const input of oldInputs) {
        if (input.name !== "TITLE") {
          console.log(`[${this.id}] updateShape_ - Removing input: ${input.name}`);
          this.removeInput(input.name, true); // true to ignore connection checks
        }
      }

      // 3. Dynamically add new fields/inputs
      //console.log(`[${this.id}] updateShape_ - Calling addFieldsRecursively with fields:`, this.messageFields);
      this.addFieldsRecursively(this.messageFields, "");

      // 4. Restore connections
      //console.log(`[${this.id}] updateShape_ - Attempting to restore ${Object.keys(savedConnections).length} connections...`);
      for (const inputName in savedConnections) {
        const targetConnection = savedConnections[inputName];
        const newInput = this.getInput(inputName);
        if (targetConnection && newInput && newInput.connection && newInput.connection.checkType_(targetConnection)) {
           try {
            console.log(`[${this.id}] updateShape_ - Restoring connection for ${inputName} to target:`, targetConnection);
            newInput.connection.connect(targetConnection);
            console.log(`[${this.id}] updateShape_ - SUCCESS restoring ${inputName}`);
           } catch (e) {
             console.warn(`[${this.id}] updateShape_ - FAILED to reconnect ${inputName}:`, e);
           }
        } else {
           console.log(`[${this.id}] updateShape_ - Skipping restore for ${inputName}. Conditions not met: target=${!!targetConnection}, newInput=${!!newInput}, checkType=${newInput?.connection?.checkType_(targetConnection ?? null)}`);
        }
      }
      //console.log(`[${this.id}] updateShape_ END`);
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
        const isNested = customMsgList.some(msg => msg.name === field.type || msg.name === `${field.type}.msg`);

        if (isNested) {
          // Recursion for subfields
          const nested = customMsgList.find(m => m.name === field.type || m.name === `${field.type}.msg`);
          if (nested && nested.fields) {
            this.addFieldsRecursively(nested.fields, fullName);
          }
        } else {
          // Create the "slot" (hole) to connect a block
          //console.log(`[${this.id}] addFieldsRecursively - Creating input FIELD_${fullName} for type ${field.type}`);
          const valueInput = this.appendValueInput(inputName)
            .appendField(fullName + ":");

          // Optionally: load a saved value (if not using shadow blocks)
          // NOTE: This "value" will not be displayed directly if it is a ValueInput,
          //       because the "editing" will come from the connected block.
          const saved = this.fieldValues[fullName] || "";

          // Adjust .setCheck(...) according to the type
          if (field.type === "string") {
            valueInput.setCheck("String");
          } else if (["int32", "int64",].includes(field.type)) {
            valueInput.setCheck("Integer"); // Use specific Integer check
          } else if (["float32", "float64"].includes(field.type)) {
            valueInput.setCheck("Float");   // Use specific Float check
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
      container.setAttribute('messagetype', this.messageType || '');
      container.setAttribute('messagefields', JSON.stringify(this.messageFields));
      container.setAttribute('fieldvalues', JSON.stringify(this.fieldValues));
      return container;
    },

    domToMutation: function (xmlElement: { getAttribute: (arg0: string) => string; }) {
      //console.log(`[${this.id}] domToMutation START`, xmlElement);
      this.messageType = xmlElement.getAttribute('messagetype') || '';

      try {
        this.messageFields = JSON.parse(xmlElement.getAttribute('messagefields') || "[]");
      } catch (e) {
        this.messageFields = [];
      }

      try {
        this.fieldValues = JSON.parse(xmlElement.getAttribute('fieldvalues') || "{}");
      } catch (e) {
        this.fieldValues = {};
      }

      // Update label if it exists
      const label = this.getField('MSG_TYPE');
      if (label) {
        label.setValue(
          this.messageType.split('.').pop()?.replace(/\.msg$/, '') || this.messageType
        );
      }

      // Now that updateShape_ preserves connections, call it here to build the block shape.
      //console.log(`[${this.id}] domToMutation - Restored state. Type: ${this.messageType}, Fields:`, this.messageFields, ` Calling updateShape_...`);
      this.updateShape_();
      //console.log(`[${this.id}] domToMutation END`);
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
      this.setColour(blockColors.Nodes);
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
      this.setColour(blockColors.Nodes);
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
      this.setColour(blockColors.Variables);
      this.setTooltip("Defines a parameter for the .msg service");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['ros2_service_block'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Service")
        .appendField(new Blockly.FieldTextInput("MyService", sanitizeNameWithoutExtension), "SERVICE_NAME");
      this.appendStatementInput("REQUEST_MESSAGES") // Just accepts message blocks
        .setCheck("ros2_named_message")
        .appendField("Request");
      this.appendDummyInput()
        .appendField("---") // Separates between Request y Response en .srv
      this.appendStatementInput("RESPONSE_MESSAGES") // Just accepts message blocks
        .setCheck("ros2_named_message")
        .appendField("Response");
        this.setColour(blockColors.Services);
      this.setTooltip("Defines a custom service in ROS with parameters.");
      this.setHelpUrl("");
    }
  };
  Blockly.Blocks['ros2_message_block'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Defines Message")
        .appendField(new Blockly.FieldTextInput("MyMessage", sanitizeNameWithoutExtension), "MESSAGE_NAME");
      this.appendStatementInput("MESSAGE_FIELDS")
        .setCheck("ros2_named_message")
        .appendField("Message fields");
        this.setColour(blockColors.Messages);
      this.setTooltip("Defines a custom messagefor ROS2.");
      this.setHelpUrl("");
    }
  };

  // On ros2-blocks.ts
  Blockly.Blocks['ros_create_server'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Create server")
        .appendField(new Blockly.FieldTextInput("MyServer", sanitizeNameWithoutExtension), "SERVER_NAME")
        this.appendDummyInput()
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
        this.setColour(blockColors.Services);
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
    this.setTooltip("Assigns a value to a custom field defined in the custom services (.srv).");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['ros2_publish_twist'] = {
  init: function () {
    this.setInputsInline(true);
    this.appendDummyInput()
      .appendField("Publish Twist from");
    this.appendDummyInput()
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");
    this.appendValueInput("LINEAR")
      .setCheck("Number")
      .appendField("Linear Velocity:");
    this.appendValueInput("ANGULAR")
      .setCheck("Number")
      .appendField("Angular Velocity:");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(blockColors.Turtlesim);
    this.setTooltip("Publishes a Twist message with user-defined linear and angular velocities and logs the message.");
    this.setHelpUrl("");
  }
};

//Block to create a client
Blockly.Blocks["ros_create_client"] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Create Client")
      this.appendDummyInput()
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
      this.appendDummyInput()
      .appendField("Server")
      .appendField(new Blockly.FieldTextInput("MyServer", sanitizeNameWithoutExtension), "SERVICE_NAME");
    this.appendDummyInput()
      .appendField("Server wait time ")
      .appendField(new Blockly.FieldNumber(0.5, 0.1, 60, 0.1), "TIMER")
      .appendField("(seconds)");
    this.appendDummyInput()
      .appendField("Server waiting message")
      .appendField(new Blockly.FieldTextInput("service not available, waiting again..."), "MESSAGE_BASE");

    // We add a "MAIN" space to connect a "child" block
    this.appendStatementInput("MAIN")
      .setCheck(null)
      .appendField("Process request");
      this.setColour(blockColors.Clients);
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

     // Esta función recorre recursivamente un bloque y sus sub-bloques
    traverseAndUpdate_: function (block: { type: string; updateFromParent: (arg0: any) => void; inputList: { connection: { targetBlock: () => any; }; }[]; nextConnection: { targetBlock: () => any; }; }, messageType: any) {
      if (!block) return;

      // Si el bloque 
      if ( block.type === 'ros_send_request') {
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
  // Notify child blocks that the clientType has changed
  updateChildren_: function () {
    const mainInput = this.getInput("MAIN");
    if (mainInput && mainInput.connection && mainInput.connection.targetBlock()) {
      const childBlock = mainInput.connection.targetBlock();
      this.traverseAndUpdate_(childBlock, this.clientType);
    }
  }
};

Blockly.Blocks["ros_send_request"] = {
  init: function () {
    this.appendDummyInput("TITLE")
      .appendField("Send request to service");
    // Block type "statement" or "void"
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(blockColors.Clients);
    this.setTooltip("Send the request and wait for the response.");
    this.setHelpUrl("");
    // Internal variables:
    this.clientType = "";       // Will be updated with "updateFromParent"
    this.requestFields = [];    // List of request fields
    this.fieldValues = {};      // *** Object to store the entered values
    this.setOnChange((event: any) => {
      if (!this.workspace || this.workspace.isDragging()) return;
    
      const parent = this.getParent(); // Obtener el bloque padre
      if (!parent) {
        // Si no se encontró un bloque 'ros_create_client' en la cadena de padres,
        // mostramos la advertencia
        this.setWarningText("This block must be inside a 'Create Client'.");
        return;
      }
    
      let warningMessage = null; // Variable para almacenar el mensaje final
      const isDynamic = !!this.clientType; // Determina si el bloque ya está configurado dinámicamente
    
      if (!isDynamic) {
        warningMessage = "This block must be inside a 'Create Client'.";
        // Si tiene inputs dinámicos, los limpiamos
        const hasDynamicInputs = this.inputList.some(
          (input: { name: string; }) => input.name?.startsWith("FIELD_")
        );
        if (hasDynamicInputs) {
          this.messageFields = [];
          this.updateShape_(); // Quita los inputs del bloque
        }
      } else {
        // Verificar si hay campos obligatorios sin conectar
        const hasUnconnectedFields = this.inputList.some(
          (input: { name: string; connection: { targetBlock: () => any; }; }) =>
            input.name?.startsWith("FIELD_") &&
            !input.connection?.targetBlock()
        );
        
        if (hasUnconnectedFields) {
          warningMessage = 'You must complete all required fields before executing.';
        }
      }
    
      // Finalmente, establecer (o limpiar) el warning
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
    console.log(`[${this.id}] domToMutation START`, xmlElement);
    // Corrected attribute names to lowercase to match XML
    this.clientType = xmlElement.getAttribute('clienttype') || "";
    try {
      this.requestFields = JSON.parse(xmlElement.getAttribute('requestfields') || "[]");
    } catch (e) {
      this.requestFields = [];
    }
    try {
      // fieldvalues seems correct in the log XML, keep it as is for now
      this.fieldValues = JSON.parse(xmlElement.getAttribute('fieldvalues') || "{}");
    } catch (e) {
      this.fieldValues = {};
    }

    // Now that updateShape_ preserves connections, call it here to build the block shape.
    console.log(`[${this.id}] domToMutation - Restored state. Type: ${this.clientType}, Fields:`, this.requestFields, ` Calling updateShape_...`);
    this.updateShape_();
    console.log(`[${this.id}] domToMutation END`);
  },

  /**
  * Called by the parent "ros_create_client" to assign the type
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
    console.log(`[${this.id}] updateShape_ START for type: ${this.clientType}`);
    // 1. Save existing connections
    const savedConnections: { [inputName: string]: Blockly.Connection | null } = {};
    for (const input of this.inputList) {
      if (input.name && input.name.startsWith("FIELD_") && input.connection && input.connection.targetConnection) {
        savedConnections[input.name] = input.connection.targetConnection;
        console.log(`[${this.id}] updateShape_ - Saved connection for ${input.name}:`, input.connection.targetConnection);
      }
    }

    // Save current field values (if any are direct fields, not value inputs)
    this.saveFieldValues();

    // 2. Remove previous dynamic inputs (except TITLE)
    const oldInputs = [...this.inputList];
    for (const inp of oldInputs) {
      if (inp.name !== "TITLE") {
        console.log(`[${this.id}] updateShape_ - Removing input: ${inp.name}`);
        this.removeInput(inp.name, true); // true to ignore connection checks
      }
    }

    // 3. Create new inputs for each request field
    console.log(`[${this.id}] updateShape_ - Adding inputs for fields:`, this.requestFields);
    for (const field of this.requestFields) {
      const inputName = `FIELD_${field.name}`;
      // Don't re-add if it somehow still exists (shouldn't with removeInput above, but safety)
      if (this.getInput(inputName)) {
         continue;
      }

      console.log(`[${this.id}] updateShape_ - Creating input ${inputName} for type ${field.type}`);
      const valueInput = this.appendValueInput(inputName)
        .appendField(field.name + ":");

      // Adjust the expected data type (setCheck)
      if (field.type === "string") {
        valueInput.setCheck("String");
      } else if (["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64"].includes(field.type)) {
        valueInput.setCheck("Integer"); // Use specific Integer check
      } else if (["float32", "float64"].includes(field.type)) {
        valueInput.setCheck("Float");   // Use specific Float check
      } else if (field.type === "bool") {
        valueInput.setCheck("Boolean");
      } else {
        valueInput.setCheck(null); // Unknown type => allow any block
      }
    }

     // 4. Restore connections
     console.log(`[${this.id}] updateShape_ - Attempting to restore ${Object.keys(savedConnections).length} connections...`);
     for (const inputName in savedConnections) {
        const targetConnection = savedConnections[inputName];
        const newInput = this.getInput(inputName);
        // Check if target is valid, input exists, and types are compatible
        if (targetConnection && newInput && newInput.connection && newInput.connection.checkType_(targetConnection)) {
           try {
              console.log(`[${this.id}] updateShape_ - Restoring connection for ${inputName} to target:`, targetConnection);
              newInput.connection.connect(targetConnection);
              console.log(`[${this.id}] updateShape_ - SUCCESS restoring ${inputName}`);
           } catch (e) {
              console.warn(`[${this.id}] updateShape_ - FAILED to reconnect ${inputName}:`, e);
           }
        } else {
           console.log(`[${this.id}] updateShape_ - Skipping restore for ${inputName}. Conditions not met: target=${!!targetConnection}, newInput=${!!newInput}, checkType=${newInput?.connection?.checkType_(targetConnection ?? null)}`);
        }
      }
    console.log(`[${this.id}] updateShape_ END`);
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
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");
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
    this.setColour(blockColors.Turtlesim);
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
    this.setColour(blockColors.Cycles);
    this.setTooltip("Pauses execution for a specified number of seconds.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['ros2_kill_turtle'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Kill turtle:")
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(blockColors.Turtlesim);
    this.setTooltip("Send a request to kill a specified turtle.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['ros2_spawn_turtle'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Spawn turtle")
      .appendField("Name:")
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");

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
    this.setColour(blockColors.Turtlesim);
    this.setTooltip("Creates a new turtle in turtlesim with name, position, and orientation.");
    this.setHelpUrl("");
  }
};


Blockly.Blocks['ros2_turtle_set_pen'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Change pen of")
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");

    this.appendValueInput("R")
      .setCheck("Number")
      .appendField("Red:");
    this.appendValueInput("G")
      .setCheck("Number")
      .appendField("Green:");
    this.appendValueInput("B")
      .setCheck("Number")
      .appendField("Blue:");
    this.appendValueInput("WIDTH")
      .setCheck("Number")
      .appendField("Width:");

    this.appendDummyInput()
      .appendField("Pen:")
      .appendField(new Blockly.FieldDropdown([
        ["Down", "0"],
        ["Up", "1"]
      ]), "PEN_STATE");

    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(blockColors.Turtlesim);
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
    this.setColour(blockColors.Turtlesim);
    this.setTooltip('Creates a timer that periodically executes a callback.');
    this.setHelpUrl('');
  }
};

Blockly.Blocks['init'] = {
  init: function () {
    this.appendDummyInput()
      .appendField('__init__')
    this.appendStatementInput('CALLBACK')
      .appendField('Execute');
    this.setPreviousStatement(false, null);
    this.setNextStatement(false, null);
    this.setColour(blockColors.Functions);
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
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");
    this.appendValueInput("GRADOS")
      .setCheck("Number")
      .appendField("Degrees:");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(blockColors.Turtlesim);
    this.setTooltip("Rotates the turtle by a specified number of degrees.");
    this.setHelpUrl("");
  }
};
// BLOCKS FOR COMMON TYPES
Blockly.Blocks['text_char_to_ascii'] = {
  init: function () {
    this.appendValueInput("CHAR")
      .setCheck("String")
      .appendField("ASCII of");

    this.setOutput(true, "Number");
    this.setColour(blockColors.Text);
    this.setTooltip("Returns the ASCII code of a single character.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['text_ascii_to_char'] = {
  init: function () {
    this.appendValueInput("ASCII_CODE")
      .appendField("Char from ASCII code");

    this.setOutput(true, "String");
    this.setColour(blockColors.Text);
    this.setTooltip("Converts an ASCII number to a character.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['float_number'] = {
  init: function () {
    this.appendDummyInput()
      .appendField(new Blockly.FieldTextInput("0.0", this.validateFloat), "NUM");
    // Output both Number (general) and Float (specific)
    this.setOutput(true, ["Number", "Float"]);
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

// Block for integer numbers
Blockly.Blocks['integer_number'] = {
  init: function() {
    this.appendDummyInput()
      .appendField(new Blockly.FieldTextInput("0", this.validateInt), "NUM");
    this.setOutput(true, ["Number", "Integer"]);
    this.setColour(230);
    this.setTooltip("Número entero explícito (int64)");
    this.setHelpUrl("");
  },

  validateInt: function(text: string) {
    const intRegex = /^-?\d+$/;
    if (!intRegex.test(text)) return null;

    return text;
  }
};

Blockly.Blocks['ros2_publish_twist_full'] = {
  init: function () {
    this.setInputsInline(false);
    this.appendDummyInput()
      .appendField("Publish Twist from")
      .appendField(new Blockly.FieldTextInput("turtle1", sanitizeBaseNameAllowUnderscoreWithoutExtension), "TURTLE_NAME");
    
    this.appendDummyInput()
      .appendField("Linear Velocity:");
    this.appendValueInput("LINEAR_X")
      .setCheck("Number")
      .appendField("linear.x:");
    this.appendValueInput("LINEAR_Y")
      .setCheck("Number")
      .appendField("linear.y:");
    this.appendValueInput("LINEAR_Z")
      .setCheck("Number")
      .appendField("linear.z:");

    this.appendDummyInput()
      .appendField("Angular Velocity:");
    this.appendValueInput("ANGULAR_X")
      .setCheck("Number")
      .appendField("angular.x:");
    this.appendValueInput("ANGULAR_Y")
      .setCheck("Number")
      .appendField("angular.y:");
    this.appendValueInput("ANGULAR_Z")
      .setCheck("Number")
      .appendField("angular.z:");

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(blockColors.Turtlesim);
    this.setTooltip("Publishes a complete Twist message, allowing full control over linear and angular vectors.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['ros2_cast_type'] = {
  init: function () {
    this.appendValueInput('VALUE')
      .setCheck(null)
      .appendField('Cast')
      .appendField(new Blockly.FieldDropdown(() => {
        const allOptionsMap = new Map<string, string>();

        common_msgs_for_custom.forEach(([label, value]) => {
          allOptionsMap.set(value, label);
        });

        customMsgList.forEach((msg) => {
          if (!allOptionsMap.has(msg.name)) {
            allOptionsMap.set(msg.name, msg.name);
          }
        });

        const options: [string, string][] = Array.from(allOptionsMap.entries()).map(
          ([value, label]) => [label, value]
        );

        return options.length > 0 ? options : [['No types available', '']];
      }), 'TARGET_TYPE');
      
    this.setOutput(true, null);
    this.setColour(blockColors.Variables);
    this.setTooltip('Casts a value to the selected ROS2 message type.');
    this.setHelpUrl('');
  }
};


function validateDescendants(block: { type: string; data: string; unplug: () => void; inputList: any[]; nextConnection: { targetBlock: () => any; }; }, selectedServiceNormalized: string) {
  if (!block) return;

  if (block.type === "srv_variable") {
    const blockService = block.data || "";
    const blockServiceNormalized = blockService.replace(/\.srv$/, "");
    if (selectedServiceNormalized === "") {
      block.unplug();
      return;
    }

    if (blockService && blockServiceNormalized !== selectedServiceNormalized) {
      messageServiceInstance?.sendMessage({
        type: 'SERVICE_MISMATCH',
        payload: {
          expected: 
          "El bloque de variable de servicio (" +
          blockServiceNormalized +
          ") no coincide con el servicio seleccionado (" +
          selectedServiceNormalized +
          ")"
        }
      });

      block.unplug();
    }
  }

  block.inputList.forEach((input) => {
    if (input.connection && input.connection.targetBlock()) {
      validateDescendants(input.connection.targetBlock(), selectedServiceNormalized);
    }
  });

  if (block.nextConnection && block.nextConnection.targetBlock()) {
    validateDescendants(block.nextConnection.targetBlock(), selectedServiceNormalized);
  }
}

