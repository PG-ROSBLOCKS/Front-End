import * as Blockly from 'blockly/core';
import {pythonGenerator, Order, PythonGenerator} from 'blockly/python';
import { removeIndentation } from '../utilities/sanitizer-tools';

const TAB_SPACE = '    '; // Tab space
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
  ['String (std_msgs)', 'std_msgs/String'],
  ['Bool (std_msgs)', 'std_msgs/Bool'],
  ['Int64 (std_msgs)', 'std_msgs/Int64'],
  ['Char (std_msgs)', 'std_msgs/Char'],
  ['Float32 (std_msgs)', 'std_msgs/Float32'],
  ['Twist (geometry_msgs)', 'geometry_msgs/Twist'],
  ['Odometry (nav_msgs)', 'nav_msgs/Odometry'],
  ['Pose (turtlesim)', 'turtlesim/Pose']
];



export function definirBloquesROS2() {
  Blockly.Blocks['ros2_create_publisher'] = {
      init: function() {
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
    init: function() {
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
    init: function() {
      this.appendDummyInput()
          .appendField("Crear suscriptor")
          .appendField(new Blockly.FieldTextInput("/mi_topico"), "TOPIC_NAME")
          .appendField(new Blockly.FieldDropdown(common_msgs), "MSG_TYPE");
  
      // C-shaped input: permite anidar más bloques dentro del callback
      this.appendStatementInput("CALLBACK")
          .setCheck(null)
          .appendField("Callback");
  
      // Se elimina la conexión previa para que no pueda unirse por un bloque superior
      // this.setPreviousStatement(true, null);
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
  
  Blockly.Blocks['ros2_print_msg_type'] = {
    init: function() {
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
    init: function() {
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

  // Services
  Blockly.Blocks['ros2_named_message'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Tipo")
          .appendField(new Blockly.FieldDropdown(common_msgs_for_custom), "MESSAGE_TYPE")
          .appendField("Nombre")
          .appendField(new Blockly.FieldTextInput("parametro"), "MESSAGE_NAME");
  
      this.setPreviousStatement(true, "ros2_named_message"); // Permite encajar con otro bloque
      this.setNextStatement(true, "ros2_named_message"); // Permite encajar con más mensajes
      this.setColour(160);
      this.setTooltip("Define un parámetro para el servicio .srv");
      this.setHelpUrl("");
    }
  };
  
  Blockly.Blocks['ros2_service_block'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Servicio")
          .appendField(new Blockly.FieldTextInput("MiServicio"), "SERVICE_NAME");
  
      this.appendStatementInput("REQUEST_MESSAGES") // Acepta solo bloques de mensajes
          .setCheck("ros2_named_message")
          .appendField("Solicitud");
  
      this.appendDummyInput()
          .appendField("---") // Indica separación entre Request y Response en .srv

      this.appendStatementInput("RESPONSE_MESSAGES") // Acepta solo bloques de mensajes
          .setCheck("ros2_named_message")
          .appendField("Respuesta");
  
      this.setColour(230);
      this.setTooltip("Define un servicio personalizado en ROS con parámetros.");
      this.setHelpUrl("");
    }
  };
  Blockly.Blocks['ros2_message_block'] = {
    init: function() {
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

  Blockly.Blocks['ros2_publish_twist'] = {
    init: function() {
      // Esta línea indica que todos los inputs se van a mostrar en la misma línea
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
    // Code generator for the block "Crear publicador"
    pythonGenerator.forBlock['ros2_create_publisher'] = function(block) {
      const topicName: string = block.getFieldValue('TOPIC_NAME');
      const msgType: string = block.getFieldValue('MSG_TYPE');

      const msgClass = addImport(msgType);

      const code = `pub_sub\n${TAB_SPACE}${TAB_SPACE}self.publisher_ = self.create_publisher(${msgClass}, '${topicName}', 10)\n`;
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
  pythonGenerator.forBlock['ros2_create_subscriber'] = function(block) {
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
  
  pythonGenerator.forBlock['ros2_subscriber_msg_data'] = function(block) {
    const code = 'msg.data';
    return [code, Order.ATOMIC];
  };

  pythonGenerator.forBlock['ros2_print_msg_type'] = function(block) {
    const code = `print("Tipo de dato recibido:", type(msg))\n`;
    return code;
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
    code += pythonGenerator.prefixLines(removeIndentation(callbackCode), `${TAB_SPACE}${TAB_SPACE}`);
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

  // Python generator for the service message block
  pythonGenerator.forBlock['ros2_named_message'] = function(block, generator) {
    var message_type = block.getFieldValue('MESSAGE_TYPE');
    var message_name = block.getFieldValue('MESSAGE_NAME');
    
    return `${message_type} ${message_name}\n`;
  };
  pythonGenerator.forBlock['ros2_service_block'] = function(block, generator) {
    var service_name = block.getFieldValue('SERVICE_NAME');
    var request_messages = generator.statementToCode(block, 'REQUEST_MESSAGES')
        .split('\n')
        .map(line => line.trim())  // Elimina espacios en cada línea
        .join('\n'); 

    var response_messages = generator.statementToCode(block, 'RESPONSE_MESSAGES')
        .split('\n')
        .map(line => line.trim())  // Elimina espacios en cada línea
        .join('\n');

    var code = `srv\n# Archivo ${service_name}.srv generado por ROSBlocks\n${request_messages}\n---\n${response_messages}`;
    return code;
  };

  pythonGenerator.forBlock['ros2_message_block'] = function(block, generator) {
    var message_name = block.getFieldValue('MESSAGE_NAME');
    var message_fields = generator.statementToCode(block, 'MESSAGE_FIELDS')
        .split('\n')
        .map(line => line.trim())  // Elimina espacios en cada línea
        .join('\n');
  
    var code = `msg\n# Archivo ${message_name}.msg generado por ROSBlocks\n${message_fields}`;
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
}
