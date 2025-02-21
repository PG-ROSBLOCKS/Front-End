import * as Blockly from 'blockly/core';
import {pythonGenerator, Order, PythonGenerator} from 'blockly/python';

const TAB_SPACE = '    '; // Tab space

export function definirBloquesROS2() {
  Blockly.Blocks['ros2_create_publisher'] = {
      init: function() {
          this.appendDummyInput()
              .appendField('Crear publicador')
              .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
              .appendField(new Blockly.FieldDropdown([
                  ['String (std_msgs)', 'std_msgs.msg.String'],
                  ['Bool (std_msgs)', 'std_msgs.msg.Bool'],
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

  //MinimalPublisher (Just for testing)
  Blockly.Blocks['ros2_minimal_publisher'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Minimal Publisher")
          .appendField("Tópico")
          .appendField(new Blockly.FieldTextInput("/mi_topico"), "TOPIC_NAME")
          .appendField("Tipo")
          .appendField(new Blockly.FieldDropdown([
            ["String (std_msgs)", "std_msgs.msg.String"],
            ["Bool (std_msgs)", "std_msgs.msg.Bool"],
            ["Int64 (std_msgs)", "std_msgs.msg.Int64"],
            ["Char (std_msgs)", "std_msgs.msg.Char"],
            ["Float32 (std_msgs)", "std_msgs.msg.Float32"],
            ["Twist (geometry_msgs)", "geometry_msgs.msg.Twist"],
            ["Odometry (nav_msgs)", "nav_msgs.msg.Odometry"],
            ["Pose (turtlesim)", "turtlesim.msg.Pose"]
          ]), "MSG_TYPE");
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
  Blockly.Blocks['ros2_minimal_subscriber'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Crear suscriptor")
          .appendField(new Blockly.FieldTextInput("/mi_topico"), "TOPIC_NAME")
          .appendField(new Blockly.FieldDropdown([
            ["String (std_msgs)", "std_msgs.msg.String"],
            ["Bool (std_msgs)", "std_msgs.msg.Bool"],
            ["Int64 (std_msgs)", "std_msgs.msg.Int64"],
            ["Char (std_msgs)", "std_msgs.msg.Char"],
            ["Float32 (std_msgs)", "std_msgs.msg.Float32"],
            ["Twist (geometry_msgs)", "geometry_msgs.msg.Twist"],
            ["Odometry (nav_msgs)", "nav_msgs.msg.Odometry"],
            ["Pose (turtlesim)", "turtlesim.msg.Pose"]
          ]), "MSG_TYPE");
      
      // C-shaped input: allows nesting more blocks inside the callback
      this.appendStatementInput("CALLBACK")
          .setCheck(null)
          .appendField("Callback");
  
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(160);
      this.setTooltip("Crea un nodo suscriptor de ROS2 con un callback para procesar mensajes entrantes.");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_subscriber_msg_data'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("respuesta al suscriptor");
      // This block returns a value (output), so we use setOutput(true)
      this.setOutput(true, "String");
      this.setColour(230);
      this.setTooltip("Devuelve el contenido de msg.data (solo válido dentro de listener_callback).");
      this.setHelpUrl("");
    }
  };
  
  
  

  // Block to publish a message
  Blockly.Blocks['ros2_publish_message'] = {
    init: function() {
      this.appendDummyInput()
          .appendField('Publicar en')
          .appendField(new Blockly.FieldTextInput('/mi_topico'), 'TOPIC_NAME')
          .appendField('tipo')
          .appendField(new Blockly.FieldDropdown([
            ['String (std_msgs)', 'std_msgs.msg.String'],
            ['Bool (std_msgs)', 'std_msgs.msg.Bool'],
            ['Int64 (std_msgs)', 'std_msgs.msg.Int64'],
            ['Char (std_msgs)', 'std_msgs.msg.Char'],
            ['Float32 (std_msgs)', 'std_msgs.msg.Float32'],
            ['Twist (geometry_msgs)', 'geometry_msgs.msg.Twist'],
            ['Odometry (nav_msgs)', 'nav_msgs.msg.Odometry'],
            ['Pose (turtlesim)', 'turtlesim.msg.Pose']
          ]), 'MSG_TYPE');
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
    init: function() {
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
          this.setColour(230);
          this.setTooltip("Registra un mensaje en el log de ROS 2 con el nivel seleccionado.");
          this.setHelpUrl("");
      }
  };

  // Block to retrieve a specific field from turtlesim.msg.Pose
Blockly.Blocks['ros2_turtlesim_pose_field'] = {
  init: function() {
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

}
type ImportsDictionary = {
  [key: string]: Set<string>;
};

// Private object with strict typing
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

// We export only the necessary functions with their types
export { addImport, getImports, clearImports };



export function definirGeneradoresROS2() {
    // Code generator for the block "Crear publicador"
    pythonGenerator.forBlock['ros2_create_publisher'] = function(block) {
      const topicName: string = block.getFieldValue('TOPIC_NAME');
      const msgType: string = block.getFieldValue('MSG_TYPE');

      const msgClass = addImport(msgType);

      const code = `${TAB_SPACE}${TAB_SPACE}self.publisher_ = self.create_publisher(${msgClass}, '${topicName}', 10)\n`;
      return code;
  };

  pythonGenerator.forBlock['ros2_minimal_publisher'] = function(block) {
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
  

  // Code generator for the block "Crear suscriptor"
  pythonGenerator.forBlock['ros2_minimal_subscriber'] = function(block) {
    const topic = block.getFieldValue('TOPIC_NAME');
    const msgType = block.getFieldValue('MSG_TYPE');
    // This is the code that the user places in the "Callback" section
    const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');
    const msgClass = addImport(msgType);
  
    let code = `${TAB_SPACE}${TAB_SPACE}self.subscription = self.create_subscription(${msgClass}, '${topic}', self.listener_callback, 10)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.subscription  # Evitar warning de variable no usada\n\n`;
    
    code += `${TAB_SPACE}def listener_callback(self, msg):\n`;
    
    // If the user does not place any code in the callback, add 'pass' to avoid indentation error
    if (!callbackCode.trim()) {
      code += `${TAB_SPACE}${TAB_SPACE}pass\n`;
    } else {
      // If there are nested blocks, insert them with proper indentation
      code += pythonGenerator.prefixLines(callbackCode, `${TAB_SPACE}${TAB_SPACE}`);
    }
  
    return code;
  };
  
  pythonGenerator.forBlock['ros2_subscriber_msg_data'] = function(block) {
    const code = 'msg.data';
    return [code, Order.ATOMIC];
  };
  
  

  // Code generator for the block "Publicar mensaje"
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


    // Code generator for the block "Crear Timer"
    pythonGenerator.forBlock['ros2_timer'] = function(block) {
        const interval = block.getFieldValue('INTERVAL');
        const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');
      
        let code = `${TAB_SPACE}${TAB_SPACE}self.timer_ = self.create_timer(${interval}, self.timer_callback)\n`;
        code += `${TAB_SPACE}def timer_callback(self):\n`;
        code += pythonGenerator.prefixLines(callbackCode, `${TAB_SPACE}${TAB_SPACE}`);
        return code;
    };

    // Code generator for the block "Log de ROS 2"
    pythonGenerator.forBlock['ros2_log'] = function(block) {
      const logLevel = block.getFieldValue('LOG_LEVEL');
      const message = pythonGenerator.valueToCode(block, 'MESSAGE', Order.NONE) || '""';

      const code = `${TAB_SPACE}${TAB_SPACE}${logLevel}(${message})\n`;
      return code;
  };  
  
    // Python generator for the turtlesim msg field block
    pythonGenerator.forBlock['ros2_turtlesim_pose_field'] = function(block) {
      const field = block.getFieldValue('FIELD');
      const code = 'msg.' + field;
      return [code, Order.ATOMIC];
  };

}
