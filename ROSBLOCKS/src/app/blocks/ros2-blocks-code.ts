import { pythonGenerator, Order } from 'blockly/python';
import * as Blockly from 'blockly/core';
import { indentSmart, removeIndentation, indentSmartByLine, indentSmartPreserveStructure } from '../utilities/sanitizer-tools';
import { srvList } from '../shared/srv-list';
import { get } from 'blockly/core/events/utils';

export const TAB_SPACE = '    '; // Tab space
export { addImport, getImports, clearImports, definirGeneradoresROS2 };

export type ImportsDictionary = {
  [key: string]: Set<string>;
};

export const importsDictMsgs: ImportsDictionary = {
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
  else if (msgType === 'time') {
    importsDictMsgs['time'] = new Set(['time']);
    return msgType;
  }

  return msgType;
}

function getImports(): string {
  let importCode = `import rclpy\nfrom rclpy.node import Node\n`;

  for (const [pkg, msgs] of Object.entries(importsDictMsgs)) {
    if (msgs.size > 0) {
      if (pkg === 'time') {
        importCode += `import time\n`;
      } else {
        importCode += `from ${pkg}.msg import ${Array.from(msgs).join(', ')}\n`;
      }
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
    mainBody; // sin indentSmartByLine
  
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
    let callbackBody = '';
    const input = block.getInput('CALLBACK');
    const targetBlock = input?.connection?.targetBlock();
    if (targetBlock) {
      const result = pythonGenerator.blockToCode(targetBlock);
      callbackBody = Array.isArray(result) ? result[0] : result;
    }
    // "Callback"
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
    if (!callbackBody.trim()) {
      code += `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}pass\n`;
    } else {
      code += pythonGenerator.prefixLines(callbackBody, `${TAB_SPACE}${TAB_SPACE}${TAB_SPACE}`);
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
    const msgType: string = block.getFieldValue('MSG_TYPE');
    const msgClass = addImport(msgType);
  
    let code = `${TAB_SPACE}${TAB_SPACE}msg = ${msgClass}()\n`;
  
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
    const turtleName = block.getFieldValue('TURTLE_NAME');
    let linear = pythonGenerator.valueToCode(block, 'LINEAR', Order.ATOMIC) || '0.0';
    let angular = pythonGenerator.valueToCode(block, 'ANGULAR', Order.ATOMIC) || '0.0';
    let code = `self.publisher_ = self.create_publisher(Twist, '/${turtleName}/cmd_vel', 10)\n`;
    code += `msg = Twist()\n`;
    code += `msg.linear.x = float(${linear})\n`;
    code += `msg.angular.z = float(${angular})\n`;
    code += `self.publisher_.publish(msg)\n`;
    code += `self.get_logger().info("Mensaje Twist publicado")\n`;

    return code;
};

  pythonGenerator.forBlock['ros2_turtle_set_pose'] = function(block) {
    const turtleName = block.getFieldValue('TURTLE_NAME');
    const x = pythonGenerator.valueToCode(block, 'X', Order.ATOMIC) || '0';
    const y = pythonGenerator.valueToCode(block, 'Y', Order.ATOMIC) || '0';
    const theta = pythonGenerator.valueToCode(block, 'THETA', Order.ATOMIC) || '0';

    const srvClass = addImport('turtlesim.srv.TeleportAbsolute');

    let code = `self.teleport_client = self.create_client(${srvClass}, '${turtleName}/teleport_absolute')\n`;
    code += `while not self.teleport_client.wait_for_service(timeout_sec=1.0): self.get_logger().info('Esperando servicio ${turtleName}/teleport_absolute...')\n`;
    code += `req = ${srvClass}.Request()\n`;
    code += `req.x = float(${x})\n`;
    code += `req.y = float(${y})\n`;
    code += `req.theta = float(${theta})\n`;
    code += `future = self.teleport_client.call_async(req)\n`;
    code += `future.add_done_callback(lambda future: self.get_logger().info('Teletransporte completado.'))\n`;
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

  pythonGenerator.forBlock['ros2_sleep'] = function (block) {
    const seconds = block.getFieldValue('SECONDS');
    addImport('time');
    const code = `time.sleep(${seconds})\n`;
    return code;
  };

  pythonGenerator.forBlock['ros2_kill_turtle'] = function(block) {
    const turtleName = block.getFieldValue('TURTLE_NAME').replace('/', '');
    const srvClass = addImport('turtlesim.srv.Kill');
    let code = `
  self.kill_client = self.create_client(${srvClass}, 'kill')
  kill_request = ${srvClass}.Request()
  kill_request.name = '${turtleName}'
  self.kill_client.call_async(kill_request)
  self.get_logger().info('Solicitud para matar a ${turtleName} enviada.')
  `;
    return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
  };
  
  pythonGenerator.forBlock['ros2_spawn_turtle'] = function(block) {
    const name = block.getFieldValue('TURTLE_NAME') || '"turtle1"';
    const x = pythonGenerator.valueToCode(block, 'X', Order.ATOMIC) || '5.0';
    const y = pythonGenerator.valueToCode(block, 'Y', Order.ATOMIC) || '5.0';
    const theta = pythonGenerator.valueToCode(block, 'THETA', Order.ATOMIC) || '0.0';

    const srvClass = addImport('turtlesim.srv.Spawn');

    let code = `self.spawn_client = self.create_client(${srvClass}, 'spawn')\n`;
    code += `while not self.spawn_client.wait_for_service(timeout_sec=1.0): self.get_logger().info('Esperando servicio spawn...')\n`;
    code += `req = ${srvClass}.Request()\n`;
    code += `req.x = float(${x})\n`;
    code += `req.y = float(${y})\n`;
    code += `req.theta = float(${theta})\n`;
    code += `req.name = '${name}'\n`;
    code += `future = self.spawn_client.call_async(req)\n`;
    code += `future.add_done_callback(lambda future: self.get_logger().info('Tortuga ' + '${name}' + ' creada en (${x}, ${y}) con orientación ${theta}.'))\n`;

    return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
};

  pythonGenerator.forBlock['ros2_turtle_set_pen'] = function(block) {
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
  self.get_logger().info('Cambiando lápiz de ${turtleName} (RGB: ${r}, ${g}, ${b}, Grosor: ${width}, Estado: ${penState}).')
  `;
      return pythonGenerator.prefixLines(code, pythonGenerator.INDENT.repeat(2));
  };

  pythonGenerator.forBlock['ros2_turtlesim_publisher'] = function (block) {
    const callbackCode = pythonGenerator.statementToCode(block, 'CALLBACK');
    addImport('time');
    let code = `pub_sub\n`;
    code += `${TAB_SPACE}${TAB_SPACE}time.sleep(1)  # Espera a que cargue el nodo\n`
    code += pythonGenerator.prefixLines(removeIndentation(callbackCode), `${TAB_SPACE}${TAB_SPACE}`);
    return code;
  };

  pythonGenerator.forBlock['ros2_turtle_rotate'] = function(block) {
    const degrees = pythonGenerator.valueToCode(block, 'GRADOS', Order.ATOMIC) || '0';
    const turtleName = block.getFieldValue('TURTLE_NAME');
    const msgClass = addImport('geometry_msgs.msg.Twist');
  
    let code = `self.publisher_ = self.create_publisher(Twist, '/${turtleName}/cmd_vel', 10)\n`;
    code += `msg = Twist()\n`;
    code += `msg.angular.z = float(${degrees})\n`;
    code += `self.publisher_.publish(msg)\n`;
    code += `self.get_logger().info("Mensaje Twist publicado")\n`;
  
    return code;
  };
}

/**
* Removes the minimum common indentation from all lines,
* preserving the difference between them (internal nesting).
*/
