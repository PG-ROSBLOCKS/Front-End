import { pythonGenerator, Order } from 'blockly/python';
import * as Blockly from 'blockly/core';
import { indentSmart, removeIndentation, removeCommonIndentation, indentSmartPreserveStructure, sanitizeCustomAtribute } from '../utilities/sanitizer-tools';
import { srvList } from '../shared/srv-list';
import { get } from 'blockly/core/events/utils';

export const TAB_SPACE = '    '; // Tab space
export { addImport, getImports, clearImports, definirGeneradoresROS2 };

export type ImportsDictionary = {
  [key: string]: Set<string>;
};

// Common ROS 2 packages
const STANDARD_ROS_PACKAGES = new Set([
  'std_msgs',
  'geometry_msgs',
  'nav_msgs',
  'sensor_msgs',
  'turtlesim'
]);

export const importsDictMsgs: ImportsDictionary = {
  'std_msgs': new Set(),
  'geometry_msgs': new Set(),
  'nav_msgs': new Set(),
  'turtlesim': new Set(),
  'custom_msgs': new Set() 
};

const importsDictSrvs: ImportsDictionary = {
  'turtlesim': new Set(),
  'custom_srvs': new Set() 
};

function addImport(msgType: string): string {
  const msgParts = msgType.split('.msg.');
  const srvParts = msgType.split('.srv.');
  console.log("Msg type: ", msgType);
  if (msgParts.length === 2) {
    const [packageName, msgName] = msgParts;
    if (STANDARD_ROS_PACKAGES.has(packageName)) {
      if (importsDictMsgs[packageName]) {
        importsDictMsgs[packageName].add(msgName);
      }
      return msgName;
    }
    return msgType;
  } else if (srvParts.length === 2) {
    const [packageName, srvName] = srvParts;
    if (STANDARD_ROS_PACKAGES.has(packageName)) {
      if (importsDictSrvs[packageName]) {
        importsDictSrvs[packageName].add(srvName);
      }
      return srvName;
    } 
    return msgType;
  }
  else if (msgType === 'time') {
    importsDictMsgs['time'] = new Set(['time']);
    return msgType;
  }
  else {
    // Without prefix, it's a custom message
    importsDictMsgs['custom_msgs'].add(msgType);
    return msgType;
  }
}

function getImports(): string {
  let importCode = `import rclpy\nfrom rclpy.node import Node\n`;
  for (const [pkg, msgs] of Object.entries(importsDictMsgs)) {
    if (msgs.size > 0) {
      if (pkg === 'time') {
        importCode += `import time\n`;
      } else if (pkg === 'custom_msgs') {
        importCode += `from sample_interfaces.msg import ${Array.from(msgs).join(', ')}\n`;
      } else {
        importCode += `from ${pkg}.msg import ${Array.from(msgs).join(', ')}\n`;
      }
    }
  }
  for (const [pkg, srvs] of Object.entries(importsDictSrvs)) {
    if (srvs.size > 0) {
      if (pkg === 'custom_srvs') {
        importCode += `from sample_interfaces.srv import ${Array.from(srvs).join(', ')}\n`;
      } else {
        importCode += `from ${pkg}.srv import ${Array.from(srvs).join(', ')}\n`;
      }
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

function definirGeneradoresROS2() {
  // Code generator for the "Create publisher" block
  pythonGenerator.forBlock['ros2_create_publisher'] = function (block) {
    const topicName = block.getFieldValue('TOPIC_NAME');
    const msgType = block.getFieldValue('MSG_TYPE');
    const msgClass = addImport(msgType);

    let mainBody = '';
    const input = block.getInput('MAIN');
    const targetBlock = input?.connection?.targetBlock();
    if (targetBlock) {
      const result = pythonGenerator.blockToCode(targetBlock);
      mainBody = Array.isArray(result) ? result[0] : result;
    }

    let code =
      `pub_sub\n` +
      `${TAB_SPACE}${TAB_SPACE}self.publisher_ = self.create_publisher(${msgClass}, '${topicName}', 10)\n` +
      mainBody; 

    code = indentSmartPreserveStructure(code, 2);
    return code;
  };

  pythonGenerator.forBlock['msg_variable'] = function (block) {
    let varName = block.getFieldValue('VAR_NAME') || '';

    // Si no empieza con "msg.", lo anteponemos
    if (!varName.startsWith('msg.')) {
      varName = 'msg.' + varName;
    }

    return [varName, Order.ATOMIC];
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

    const msgClass = addImport(msgType);

    const callbackBlock = block.getInputTargetBlock('CALLBACK');
    let callbackCode: string = '';

    if (callbackBlock) {
      const generated = pythonGenerator.blockToCode(callbackBlock);
      if (Array.isArray(generated)) {
        callbackCode = generated[0];
      } else {
        callbackCode = generated;
      }
    }

    let code = 'pub_sub\n';
    code += `${TAB_SPACE}${TAB_SPACE}self.subscription = self.create_subscription(${msgClass}, '${topic}', self.listener_callback, 10)\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.subscription\n\n`;

    code += `${TAB_SPACE}def listener_callback(self, msg):\n`;

    if (!callbackCode.trim()) {
      code += `${TAB_SPACE}${TAB_SPACE}pass\n`;
    } else {
      code += pythonGenerator.prefixLines(callbackCode, TAB_SPACE.repeat(2));
    }
    console.log(code);
    return code;
  };


  pythonGenerator.forBlock['ros2_subscriber_msg_data'] = function (block) {
    const code = 'msg.data';
    return [code, Order.ATOMIC];
  };

  pythonGenerator.forBlock['ros2_print_msg_type'] = function (block) {
    const varCode = pythonGenerator.valueToCode(block, 'VAR_EXPR', Order.NONE) || 'msg';
    const code = `print("Tipo de dato recibido:", type(${varCode}))\n`;
    return code;
  };
  
  
  

  // Code generator for the block "Publicar mensaje"
  pythonGenerator.forBlock['ros2_publish_message'] = function (block) {
    const msgType: string = block.getFieldValue('MSG_TYPE');

    let code = `${TAB_SPACE}${TAB_SPACE}msg = ${msgType}()\n`;

    // Recorre la lista de inputs con block.inputList o conoces sus nombres
    for (const input of block.inputList) {
      if (input.name && input.name.startsWith("FIELD_")) {
        const fieldName = input.name.replace("FIELD_", ""); // "pose.position.x"
        const valueCode = pythonGenerator.valueToCode(
          block,
          input.name,
          Order.NONE
        ) || '""'; // valor por defecto

        code += `msg.${fieldName} = ${valueCode}\n`;
      }
    }

    code += `self.publisher_.publish(msg)\n`;
    return code;
  };

  // Code generator for the block "Crear Timer"
  pythonGenerator.forBlock['ros2_timer'] = function (block) {
    const interval = block.getFieldValue('INTERVAL');
    const callbackBody = pythonGenerator.statementToCode(block, 'CALLBACK');

    let code = `self.timer_ = self.create_timer(${interval}, self.timer_callback)\n`;
    code += `def timer_callback(self):\n`;
    code += callbackBody;

    return code;
  };


  // Code generator for the block "Log de ROS 2"
  pythonGenerator.forBlock['ros2_log'] = function (block) {
    const logLevel = block.getFieldValue('LOG_LEVEL');
    const message = pythonGenerator.valueToCode(block, 'MESSAGE', Order.NONE) || '""';

    const code = `${logLevel}(str(${message}))\n`;
    return code;
  };

  // Python generator for the service message block
  pythonGenerator.forBlock['ros2_named_message'] = function (block, generator) {
    var message_type = block.getFieldValue('MESSAGE_TYPE');
    var message_name = block.getFieldValue('MESSAGE_NAME');

    return `${message_type} ${sanitizeCustomAtribute(message_name)}\n`;
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
    callbackCode = removeIndentation(callbackCode);
  
    let code =
      `server|${serviceType}\n` +
      `${TAB_SPACE}${TAB_SPACE}self.service_ = self.create_service(${serviceType}, '${serviceName}', self.service_callback)\n\n` +
      `${TAB_SPACE}def service_callback(self, request, response):\n` +
      `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}${callbackCode}\n` +
      `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}return response\n`;

    code = indentSmartPreserveStructure(code, 2);
    return code;
  };
  

  pythonGenerator.forBlock['ros2_publish_twist'] = function (block) {
    const msgClass = addImport('geometry_msgs.msg.Twist');
    const turtleName = block.getFieldValue('TURTLE_NAME');
    let linear = pythonGenerator.valueToCode(block, 'LINEAR', Order.ATOMIC) || '0.0';
    let angular = pythonGenerator.valueToCode(block, 'ANGULAR', Order.ATOMIC) || '0.0';
    let code = `self.publisher_ = self.create_publisher(Twist, '/${turtleName}/cmd_vel', 10)\n`;
    code += `msg = Twist()\n`;
    code += `msg.linear.x = float(${linear})\n`;
    code += `msg.angular.z = float(${angular})\n`;
    code += `self.publisher_.publish(msg)\n`;
    code += `self.get_logger().info("Twist message published")\n`;

    return code;
  };

  pythonGenerator.forBlock['ros2_turtle_set_pose'] = function (block) {
    const turtleName = block.getFieldValue('TURTLE_NAME');
    const x = pythonGenerator.valueToCode(block, 'X', Order.ATOMIC) || '0';
    const y = pythonGenerator.valueToCode(block, 'Y', Order.ATOMIC) || '0';
    const theta = pythonGenerator.valueToCode(block, 'THETA', Order.ATOMIC) || '0';

    const srvClass = addImport('turtlesim.srv.TeleportAbsolute');

    let code = `self.teleport_client = self.create_client(${srvClass}, '${turtleName}/teleport_absolute')\n`;
    code += `while not self.teleport_client.wait_for_service(timeout_sec=1.0): self.get_logger().info('Waiting service ${turtleName}/teleport_absolute...')\n`;
    code += `req = ${srvClass}.Request()\n`;
    code += `req.x = float(${x})\n`;
    code += `req.y = float(${y})\n`;
    code += `req.theta = float(${theta})\n`;
    code += `future = self.teleport_client.call_async(req)\n`;
    code += `future.add_done_callback(lambda future: self.get_logger().info('Teleport completed.'))\n`;
    code += `self.get_logger().info('Turtle in (${x}, ${y}) and orientation ${theta}.')\n`;

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

    clientType = clientType.replace('.srv', '');
    // 3) Generates class and construction
    // Start building the generated code; the first line is to identify the block type
    let code = `client|${clientType}\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.cli = self.create_client(${clientType}, '${serviceName}')\n`;
    code += `${TAB_SPACE}${TAB_SPACE}while not self.cli.wait_for_service(timeout_sec=${timer}):\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}#STARTWHILE\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}self.get_logger().info('${messageBase}')\n`;
    code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}#ENDWHILE\n`;
    code += `${TAB_SPACE}${TAB_SPACE}self.req = ${clientType}.Request()\n\n`;
    const paramList = requestFields.map(field => field.name).join(', ');
    code += `${TAB_SPACE}def send_request(self, ${paramList}):\n`;
    // 5) Assign each field in self.req.<field> = <field>
    requestFields.forEach(field => {
      code += `${TAB_SPACE}${TAB_SPACE}self.req.${field.name} = ${field.name}\n`;
    });
    // 6) Return asynchronous call
    code += `${TAB_SPACE}${TAB_SPACE}return self.cli.call_async(self.req)\n`;
    code += `#main-sendrequest\n`;
    console.log(code);
    console.log("---------------------------------")
    let main_code = pythonGenerator.statementToCode(block, 'MAIN');
    console.log(main_code);
    console.log("---------------------------------")
    code += main_code
    code = indentSmartPreserveStructure(code, 2);
    console.log(code);
    return code;
  };

  pythonGenerator.forBlock["ros_send_request"] = function (block, generator) {
    const myBlock = block as any;
    const clientType = myBlock.clientType || "UnknownSrv";
    const requestFields = myBlock.requestFields || [];
    let values: any[] = [];
    let assignments = "";
  
    for (const field of requestFields) {
      const inputName = `FIELD_${field.name}`;
  
      // Obtener el código conectado al input dinámico
      const valueCode =
        generator.valueToCode(block, inputName, Order.NONE) || "None";
  
      assignments += `self.req.${field.name} = ${valueCode}\n`;
      values.push(valueCode);
    }
  
    let code = "";
    code += assignments;

    let request =  `future = node.send_request(${values.join(', ')})\n`;
    code += request;

    code += `rclpy.spin_until_future_complete(node, future)\n`;
    code += `response = future.result()\n`;
    return code;
  };
  

  pythonGenerator.forBlock['ros2_sleep'] = function (block) {
    const seconds = block.getFieldValue('SECONDS');
    addImport('time');
    const code = `time.sleep(${seconds})\n`;
    return code;
  };

  pythonGenerator.forBlock['ros2_kill_turtle'] = function (block) {
    const turtleName = block.getFieldValue('TURTLE_NAME').replace('/', '');
    const srvClass = addImport('turtlesim.srv.Kill');
    let code = `
  self.kill_client = self.create_client(${srvClass}, 'kill')
  kill_request = ${srvClass}.Request()
  kill_request.name = '${turtleName}'
  self.kill_client.call_async(kill_request)
  self.get_logger().info('Request to kill ${turtleName} sent.')
  `;
    return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
  };

  pythonGenerator.forBlock['ros2_spawn_turtle'] = function (block) {
    const name = block.getFieldValue('TURTLE_NAME') || 'turtle1';
    const x = pythonGenerator.valueToCode(block, 'X', Order.ATOMIC) || '5.0';
    const y = pythonGenerator.valueToCode(block, 'Y', Order.ATOMIC) || '5.0';
    const theta = pythonGenerator.valueToCode(block, 'THETA', Order.ATOMIC) || '0.0';
  
    const srvClass = addImport('turtlesim.srv.Spawn');
  
    let code = `self.spawn_client = self.create_client(${srvClass}, 'spawn')\n`;
    code += `while not self.spawn_client.wait_for_service(timeout_sec=1.0): self.get_logger().info('Waiting for spawn service...')\n`;
    code += `req = ${srvClass}.Request()\n`;
    code += `req.x = float(${x})\n`;
    code += `req.y = float(${y})\n`;
    code += `req.theta = float(${theta})\n`;
    code += `req.name = ${JSON.stringify(name)}\n`;
    code += `future = self.spawn_client.call_async(req)\n`;
    code += `future.add_done_callback(lambda future: self.get_logger().info(f"Turtle {req.name} created in (${x}, ${y}) with orientation ${theta}."))\n`;
  
    return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
  };
  

  pythonGenerator.forBlock['ros2_turtle_set_pen'] = function (block) {
    const turtleName = block.getFieldValue('TURTLE_NAME');
    const r = pythonGenerator.valueToCode(block, 'R', Order.ATOMIC) || '0';
    const g = pythonGenerator.valueToCode(block, 'G', Order.ATOMIC) || '0';
    const b = pythonGenerator.valueToCode(block, 'B', Order.ATOMIC) || '0';
    const width = pythonGenerator.valueToCode(block, 'WIDTH', Order.ATOMIC) || '1';
    const penState = block.getFieldValue('PEN_STATE'); // 0 = abajo, 1 = arriba

    const srvClass = addImport('turtlesim.srv.SetPen');

    let code = `
  self.pen_client = self.create_client(${srvClass}, '${turtleName}/set_pen')
  pen_request = ${srvClass}.Request()
  pen_request.r = int(${r})
  pen_request.g = int(${g})
  pen_request.b = int(${b})
  pen_request.width = int(${width})
  pen_request.off = int(${penState})  # 0 = lápiz abajo, 1 = lápiz arriba
  self.pen_client.call_async(pen_request)
  self.get_logger().info('Changing pen ${turtleName} (RGB: ${r}, ${g}, ${b}, Width: ${width}, Status: ${penState}).')
  `;
    return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
  };

  pythonGenerator.forBlock['ros2_turtlesim_publisher'] = function (block) {
    addImport('time');

    let mainBody = '';
    const input = block.getInput('CALLBACK');
    const targetBlock = input?.connection?.targetBlock();
    if (targetBlock) {
      const result = pythonGenerator.blockToCode(targetBlock);
      mainBody = Array.isArray(result) ? result[0] : result;
    }

    let code =
      `pub_sub\n` +
      `${TAB_SPACE}${TAB_SPACE}time.sleep(2)\n` +
      mainBody; 

    code = indentSmartPreserveStructure(code, 2);
    return code;
  };

  pythonGenerator.forBlock['init'] = function (block) {
    addImport('time');

    let mainBody = '';
    const input = block.getInput('CALLBACK');
    const targetBlock = input?.connection?.targetBlock();
    if (targetBlock) {
      const result = pythonGenerator.blockToCode(targetBlock);
      mainBody = Array.isArray(result) ? result[0] : result;
    }

    let code =
      `pub_sub\n` +
      `${TAB_SPACE}${TAB_SPACE}time.sleep(2)\n` +
      mainBody; 

    code = indentSmartPreserveStructure(code, 2);
    return code;
  };

  pythonGenerator.forBlock['ros2_turtle_rotate'] = function (block) {
    const degrees = pythonGenerator.valueToCode(block, 'GRADOS', Order.ATOMIC) || '0';
    const turtleName = block.getFieldValue('TURTLE_NAME');
    const msgClass = addImport('geometry_msgs.msg.Twist');

    let code = `self.publisher_ = self.create_publisher(Twist, '/${turtleName}/cmd_vel', 10)\n`;
    code += `msg = Twist()\n`;
    code += `msg.angular.z = float(${degrees})\n`;
    code += `self.publisher_.publish(msg)\n`;
    code += `self.get_logger().info("Twist message published")\n`;

    return code;
  };

  //override for if in order to comment the start and the end of the if block
  pythonGenerator.forBlock['controls_if'] = function (block) {
    // Inicializamos variables para generar el código.
    let n = 0;
    let code = '';
    let branchCode = '';

    // Procesa la parte "if" y las cláusulas "elif"
    do {
      // Obtiene el código de la condición.
      const conditionCode = pythonGenerator.valueToCode(block, 'IF' + n, Order.NONE) || 'False';
      // Obtiene el código del cuerpo asociado a la condición.
      branchCode = pythonGenerator.statementToCode(block, 'DO' + n);
      // Agrega los marcadores al inicio y al final del bloque if/elif.
      branchCode = "#STARTIF\n" + branchCode + "#ENDIF\n";
      // Usa "if" para la primera condición y "elif" para las siguientes.
      code += (n === 0 ? "if " : "elif ") + conditionCode + ":\n" + branchCode;
      n++;
    } while (block.getInput('IF' + n));

    // Procesa la parte "else" si existe.
    if (block.getInput('ELSE')) {
      branchCode = pythonGenerator.statementToCode(block, 'ELSE');
      branchCode = "#STARTELSE\n" + branchCode + "#ENDELSE\n";
      code += "else:\n" + branchCode;
    }
    return code;
  };

  //override for if else in order to comment the start and the end of the if else block
  pythonGenerator.forBlock['controls_ifelse'] = function (block) {
    // Inicializamos variables para generar el código.
    let n = 0;
    let code = '';
    let branchCode = '';

    // Procesa la parte "if" y las cláusulas "elif"
    do {
      // Obtiene el código de la condición.
      const conditionCode = pythonGenerator.valueToCode(block, 'IF' + n, Order.NONE) || 'False';
      // Obtiene el código del cuerpo asociado a la condición.
      branchCode = pythonGenerator.statementToCode(block, 'DO' + n);
      // Agrega los marcadores al inicio y al final del bloque if/elif.
      branchCode = "#STARTIF\n" + branchCode + "#ENDIF\n";
      // Usa "if" para la primera condición y "elif" para las siguientes.
      code += (n === 0 ? "if " : "elif ") + conditionCode + ":\n" + branchCode;
      n++;
    } while (block.getInput('IF' + n));

    // Procesa la parte "else" si existe.
    if (block.getInput('ELSE')) {
      branchCode = pythonGenerator.statementToCode(block, 'ELSE');
      branchCode = "#STARTELSE\n" + branchCode + "#ENDELSE\n";
      code += "else:\n" + branchCode;
    }
    return code;
  }

  //override de controls_repeat in order to comment the start and the end of the repeat block
  pythonGenerator.forBlock['controls_repeat'] = function (block) {
    const repeats = block.getFieldValue('TIMES') || '0'; // Ensure 'TIMES' is retrieved correctly
    let branch = pythonGenerator.statementToCode(block, 'DO');
    branch = "#STARTREPEAT\n" + branch + "#ENDREPEAT\n";
    return `for _ in range(${repeats}):\n${branch}`
  }

  //override for repeat in order to comment the start and the end of the repeat block
  pythonGenerator.forBlock['controls_repeat_ext'] = function (block) {
    const repeats = pythonGenerator.valueToCode(block, 'TIMES', Order.NONE) || '0';
    let branch = pythonGenerator.statementToCode(block, 'DO');
    branch = "#STARTREPEAT\n" + branch + "#ENDREPEAT\n";
    return `for _ in range(${repeats}):\n${branch}`;
  }

  //override for while in order to comment the start and the end of the while block
  pythonGenerator.forBlock['controls_whileUntil'] = function (block) {
    const until = block.getFieldValue('MODE') === 'UNTIL';
    const argument0 = pythonGenerator.valueToCode(block, 'BOOL', until ? Order.LOGICAL_NOT : Order.NONE) || 'False';
    let branch = pythonGenerator.statementToCode(block, 'DO');
    branch = "#STARTWHILE\n" + branch + "#ENDWHILE\n";
    return `${until ? 'while not ' : 'while '}${argument0}:\n${branch}`;
  };

  //override for for in order to comment the start and the end of the for block
  pythonGenerator.forBlock['controls_for'] = function (block) {
    const variable0 = pythonGenerator.nameDB_?.getName(
      block.getFieldValue('VAR'), Blockly.VARIABLE_CATEGORY_NAME);
    const argument0 = pythonGenerator.valueToCode(block, 'FROM', Order.NONE) || '0';
    const argument1 = pythonGenerator.valueToCode(block, 'TO', Order.NONE) || '0';
    const increment = pythonGenerator.valueToCode(block, 'BY', Order.NONE) || '1';
    let branch = pythonGenerator.statementToCode(block, 'DO');
    branch = "#STARTFOR\n" + branch + "#ENDFOR\n";
    let code;
    let inc;
    if (argument0.match(/\d+/) && argument1.match(/\d+/) && increment.match(/\d+/)) {
      inc = parseInt(increment);
      // Se elimina el "+ 1" para que el rango vaya de argument0 a argument1-1 según la semántica de range()
      code = `for ${variable0} in range(${argument0}, ${argument1}, ${inc}):\n${branch}`;
    } else {
      code = `for ${variable0} in range(${argument0}, ${argument1}):\n${branch}`;
    }
    return code;
  }
  

  //override the foreach block in order to comment the start and the end of the foreach block
  pythonGenerator.forBlock['controls_forEach'] = function (block) {
    const variable0 = pythonGenerator.nameDB_?.getName(
      block.getFieldValue('VAR'), Blockly.VARIABLE_CATEGORY_NAME);
    const argument0 = pythonGenerator.valueToCode(block, 'LIST', Order.NONE) || '[]';
    let branch = pythonGenerator.statementToCode(block, 'DO');
    branch = "#STARTFOREACH\n" + branch + "#ENDFOREACH\n";
    return `for ${variable0} in ${argument0}:\n${branch}`;
  }

  // BLOCKS FOR COMMON TYPES
  pythonGenerator.forBlock['text_char_to_ascii'] = function (block) {
    const char = block.getFieldValue('CHAR') || 'a';
    const code = `ord('${char}')`;
    return [code, Order.ATOMIC];
  };
  pythonGenerator.forBlock['text_ascii_to_char'] = function (block) {
    const value = pythonGenerator.valueToCode(block, 'ASCII_CODE', Order.NONE) || '0';
    const code = `chr(${value})`;
    return [code, Order.FUNCTION_CALL];
  };
  
  pythonGenerator.forBlock['float_number'] = function(block) {
  const code = block.getFieldValue('NUM');

  // Ya está validado como float con punto decimal y precisión adecuada
  return [code, Order.ATOMIC];
};
}

pythonGenerator.forBlock['ros2_publish_twist_full'] = function (block) {
  const msgClass = addImport('geometry_msgs.msg.Twist');
  const turtleName = block.getFieldValue('TURTLE_NAME');

  const linearX = pythonGenerator.valueToCode(block, 'LINEAR_X', Order.ATOMIC) || '0.0';
  const linearY = pythonGenerator.valueToCode(block, 'LINEAR_Y', Order.ATOMIC) || '0.0';
  const linearZ = pythonGenerator.valueToCode(block, 'LINEAR_Z', Order.ATOMIC) || '0.0';
  const angularX = pythonGenerator.valueToCode(block, 'ANGULAR_X', Order.ATOMIC) || '0.0';
  const angularY = pythonGenerator.valueToCode(block, 'ANGULAR_Y', Order.ATOMIC) || '0.0';
  const angularZ = pythonGenerator.valueToCode(block, 'ANGULAR_Z', Order.ATOMIC) || '0.0';

  let code = `self.publisher_ = self.create_publisher(${msgClass}, '/${turtleName}/cmd_vel', 10)\n`;
  code += `msg = ${msgClass}()\n`;
  code += `msg.linear.x = float(${linearX})\n`;
  code += `msg.linear.y = float(${linearY})\n`;
  code += `msg.linear.z = float(${linearZ})\n`;
  code += `msg.angular.x = float(${angularX})\n`;
  code += `msg.angular.y = float(${angularY})\n`;
  code += `msg.angular.z = float(${angularZ})\n`;
  code += `self.publisher_.publish(msg)\n`;
  code += `self.get_logger().info("Published full Twist message")\n`;

  return code;
};

pythonGenerator.forBlock['integer_number'] = function (block) {
  const numberText = block.getFieldValue('NUM') || '0';
  return [numberText, Order.ATOMIC];
};


pythonGenerator.forBlock['ros2_cast_type'] = function (block) {
  const valueCode = pythonGenerator.valueToCode(block, 'VALUE', Order.NONE) || 'None';
  const targetType = block.getFieldValue('TARGET_TYPE') || '';

  let castedCode = valueCode;

  if (targetType) {
    if (['string', 'bool', 'int64', 'int16', 'float32', 'float64', 'char', 'byte'].includes(targetType)) {
      // Para tipos primitivos: usar funciones estándar de Python
      if (targetType === 'string') {
        castedCode = `str(${valueCode})`;
      } else if (targetType === 'bool') {
        castedCode = `bool(${valueCode})`;
      } else if (targetType === 'char') {
        castedCode = `chr(${valueCode})`;
      } else if (targetType === 'byte') {
        castedCode = `bytes([${valueCode}])`; // Byte único
      } else {
        // int64, int16, float32, float64
        castedCode = `${targetType.startsWith('float') ? 'float' : 'int'}(${valueCode})`;
      }
    } else {
      // Para tipos de ROS2 complejos: crear una instancia
      // Ejemplo: Twist(), Odometry(), Pose()
      const typeName = targetType.split('/').pop() || targetType;
      castedCode = `${typeName}()`;
    }
  }

  return [castedCode, Order.FUNCTION_CALL];
};

pythonGenerator.forBlock['integer_number'] = function (block) {
  const numberText = block.getFieldValue('NUM') || '0';
  return [numberText, Order.ATOMIC];
};