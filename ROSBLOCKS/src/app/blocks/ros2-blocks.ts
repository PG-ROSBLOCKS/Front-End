import * as Blockly from 'blockly/core';
import {pythonGenerator, Order, PythonGenerator} from 'blockly/python';

const TAB_SPACE = '    '; // Espacio de tabulación

export function definirBloquesROS2() {
  Blockly.Blocks['ros2_create_publisher'] = {
      init: function() {
          this.appendDummyInput()
              .appendField('Crear publicador')
              .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
              .appendField(new Blockly.FieldDropdown([
                  ['Bool (std_msgs)', 'std_msgs.msg.Bool'],
                  ['String (std_msgs)', 'std_msgs.msg.String'],
                  ['Int64 (std_msgs)', 'std_msgs.msg.Int64'],
                  ['Char (std_msgs)', 'std_msgs.msg.Char'],
                  ['Float32 (std_msgs)', 'std_msgs.msg.Float32'],
                  ['Twist (geometry_msgs)', 'geometry_msgs.msg.Twist'],
                  ['Odometry (nav_msgs)', 'nav_msgs.msg.Odometry'],
                  ['Pose (turtlesim)', 'turtlesim.msg.Pose']
              ]), 'MSG_TYPE');
          this.setPreviousStatement(true, null);
          this.setNextStatement(true, null);
          this.setColour(160);
      }
  };

  // Bloque para crear un suscriptor
  Blockly.Blocks['ros2_create_subscriber'] = {
    init: function() {
      this.appendDummyInput()
          .appendField('Crear suscriptor')
          .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
          .appendField(new Blockly.FieldDropdown([
            ['std_msgs/String', 'String'],
            ['geometry_msgs/Twist', 'Twist'],
            ['sensor_msgs/Image', 'Image']
          ]), 'MSG_TYPE');
      this.appendStatementInput('CALLBACK')
          .appendField('Callback');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip('Crea un suscriptor para un tópico de ROS 2.');
      this.setHelpUrl('');
    }
  };
  // Bloque para publicar un mensaje
  Blockly.Blocks['ros2_publish_message'] = {
    init: function() {
      this.appendDummyInput()
          .appendField('Publicar en')
          .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
          .appendField('tipo')
          .appendField(new Blockly.FieldDropdown([
            ['std_msgs/String', 'String'],
            ['geometry_msgs/Twist', 'Twist'],
            ['sensor_msgs/Image', 'Image']
          ]), 'MSG_TYPE');
      this.appendDummyInput()
          .appendField('Mensaje')
          .appendField(new Blockly.FieldTextInput('Hola, ROS 2!'), 'MESSAGE_CONTENT');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160); // Color del bloque
      this.setTooltip('Publica un mensaje en un tópico de ROS 2.');
      this.setHelpUrl('');
    }
  };
  // Bloque para crear un timer
  Blockly.Blocks['ros2_timer'] = {
    init: function() {
      this.appendDummyInput()
          .appendField('Timer cada')
          .appendField(new Blockly.FieldNumber(1, 0.1, 60, 0.1), 'INTERVAL') // Intervalo en segundos
          .appendField('segundos');
      this.appendStatementInput('CALLBACK') // Bloque de código para el callback
          .appendField('Ejecutar');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(120); // Color del bloque
      this.setTooltip('Crea un timer que ejecuta un callback periódicamente.');
      this.setHelpUrl('');
    }
  };
  // Bloque para log de ROS 2
  Blockly.Blocks['ros2_log'] = {
      init: function() {
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
          this.setColour(230); // Color del bloque
          this.setTooltip("Registra un mensaje en el log de ROS 2 con el nivel seleccionado.");
          this.setHelpUrl("");
      }
  };
}
type ImportsDictionary = {
  [key: string]: Set<string>;
};

//  Objeto privado con tipado estricto
const importsDict: ImportsDictionary = {
  'std_msgs': new Set(),
  'geometry_msgs': new Set(),
  'nav_msgs': new Set(),
  'turtlesim': new Set()
};

function addImport(msgType: string): string {
  const parts = msgType.split('.msg.');
  if (parts.length === 2) {
      const packageName = parts[0];
      const msgName = parts[1];

      if (importsDict[packageName]) {
          importsDict[packageName].add(msgName);
      }
      return msgName;
  }
  return msgType;
}


function getImports(): string {
  let importCode = `import rclpy\nfrom rclpy.node import Node\n`;

  for (const [pkg, msgs] of Object.entries(importsDict)) {
      if (msgs.size > 0) {
          importCode += `from ${pkg}.msg import ${Array.from(msgs).join(', ')}\n`;
      }
  }

  return importCode + '\n';
}

function clearImports(): void {
  for (const key in importsDict) {
      importsDict[key].clear();
  }
}

// Exportamos solo las funciones necesarias con su tipado
export { addImport, getImports, clearImports };



export function definirGeneradoresROS2() {
    // Generador de código para el bloque "Crear publicador"
    pythonGenerator.forBlock['ros2_create_publisher'] = function(block) {
      const topicName: string = block.getFieldValue('TOPIC_NAME');
      const msgType: string = block.getFieldValue('MSG_TYPE');

      const msgClass = addImport(msgType);

      const code = `${TAB_SPACE}${TAB_SPACE}self.publisher_ = self.create_publisher(${msgClass}, '${topicName}', 10)\n`;
      return code;
  };

  // Generador de código para el bloque "Crear suscriptor"
  pythonGenerator.forBlock['ros2_create_subscriber'] = function(block) {
      const topicName: string = block.getFieldValue('TOPIC_NAME');
      const msgType: string = block.getFieldValue('MSG_TYPE');
      const callbackCode: string = pythonGenerator.statementToCode(block, 'CALLBACK');

      const msgClass = addImport(msgType);

      const code = `${TAB_SPACE}${TAB_SPACE}self.subscription_ = self.create_subscription(${msgClass}, '${topicName}', ${callbackCode}, 10)\n`;
      return code;
  };

  // Generador de código para el bloque "Publicar mensaje"
  pythonGenerator.forBlock['ros2_publish_message'] = function(block) {
      const topicName: string = block.getFieldValue('TOPIC_NAME');
      const msgType: string = block.getFieldValue('MSG_TYPE');
      const messageContent: string = block.getFieldValue('MESSAGE_CONTENT');

      const msgClass = addImport(msgType);

      let code = `${TAB_SPACE}${TAB_SPACE}msg = ${msgClass}()\n`;
      code += `${TAB_SPACE}${TAB_SPACE}msg.data = "${messageContent}"\n`;
      code += `${TAB_SPACE}${TAB_SPACE}self.publisher_.publish(msg)\n`;
      return code;
  };


    // Generador de código para el bloque "Crear Timer"
    pythonGenerator.forBlock['ros2_timer'] = function(block) {
        const interval = block.getFieldValue('INTERVAL'); // Obtener el intervalo del timer
        const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK'); // Obtener el código del callback
      
        let code = `${TAB_SPACE}${TAB_SPACE}self.timer_ = self.create_timer(${interval}, self.timer_callback)\n`;
        code += `${TAB_SPACE}def timer_callback(self):\n`;
        code += pythonGenerator.prefixLines(callbackCode, `${TAB_SPACE}${TAB_SPACE}`); // Indentar correctamente el callback
        return code;
    };

    // Generador de código para el bloque "Log de ROS 2"
    pythonGenerator.forBlock['ros2_log'] = function(block) {
      const logLevel = block.getFieldValue('LOG_LEVEL');
      const message = pythonGenerator.valueToCode(block, 'MESSAGE', Order.NONE) || '""';

      const code = `${TAB_SPACE}${TAB_SPACE}${logLevel}(${message})\n`;
      return code;
  };    
}
