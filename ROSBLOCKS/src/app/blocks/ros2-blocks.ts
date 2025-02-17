import * as Blockly from 'blockly/core';
import {pythonGenerator, Order, PythonGenerator} from 'blockly/python';


export function definirBloquesROS2() {
  // Bloque para crear un publicador
  Blockly.Blocks['ros2_create_publisher'] = {
    init: function() {
      this.appendDummyInput()
          .appendField('Crear publicador')
          .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
          .appendField(new Blockly.FieldDropdown([
            ['std_msgs/String', 'std_msgs.msg.String'],
            ['geometry_msgs/Twist', 'geometry_msgs.msg.Twist'],
            ['sensor_msgs/Image', 'sensor_msgs.msg.Image']
          ]), 'MSG_TYPE');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip('Crea un publicador para un tópico de ROS 2.');
      this.setHelpUrl('');
    }
  };

  // Bloque para crear un suscriptor
  Blockly.Blocks['ros2_create_subscriber'] = {
    init: function() {
      this.appendDummyInput()
          .appendField('Crear suscriptor')
          .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
          .appendField(new Blockly.FieldDropdown([
            ['std_msgs/String', 'std_msgs.msg.String'],
            ['geometry_msgs/Twist', 'geometry_msgs.msg.Twist'],
            ['sensor_msgs/Image', 'sensor_msgs.msg.Image']
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
            ['std_msgs/String', 'std_msgs.msg.String'],
            ['geometry_msgs/Twist', 'geometry_msgs.msg.Twist'],
            ['sensor_msgs/Image', 'sensor_msgs.msg.Image']
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
}

export function definirGeneradoresROS2() {
    // Generador de código para el bloque "Crear publicador"
    pythonGenerator.forBlock['ros2_create_publisher'] = function(block) {
      const topicName = block.getFieldValue('TOPIC_NAME');
      const msgType = block.getFieldValue('MSG_TYPE');
      const code = `self.publisher_ = self.create_publisher(${msgType}, '${topicName}', 10)\n`;
      return code;
    };
  
    // Generador de código para el bloque "Crear suscriptor"
    pythonGenerator.forBlock['ros2_create_subscriber'] = function(block) {
      const topicName = block.getFieldValue('TOPIC_NAME');
      const msgType = block.getFieldValue('MSG_TYPE');
      const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');
      const code = `self.subscription_ = self.create_subscription(${msgType}, '${topicName}', ${callbackCode}, 10)\n`;
      return code;
    };
  
    pythonGenerator.forBlock['ros2_publish_message'] = function(block) {
        const topicName = block.getFieldValue('TOPIC_NAME'); // Obtener el nombre del tópico
        const msgType = block.getFieldValue('MSG_TYPE'); // Obtener el tipo de mensaje
        const messageContent = block.getFieldValue('MESSAGE_CONTENT'); // Obtener el contenido del mensaje
        let code = `msg = ${msgType}()\n`;
        code += `msg.data = "${messageContent}"\n`;
        code += `self.publisher_.publish(msg)\n`;
        return code;
    };
    pythonGenerator.forBlock['ros2_timer'] = function(block) {
        const interval = block.getFieldValue('INTERVAL'); // Obtener el intervalo del timer
        const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK'); // Obtener el código del callback
      
        // Generar el código Python para el timer
        let code = `self.timer_ = self.create_timer(${interval}, self.timer_callback)\n`;
        code += `def timer_callback(self):\n`;
        code += pythonGenerator.prefixLines(callbackCode, '    '); // Indentar el código del callback
        return code;
      };
  }
