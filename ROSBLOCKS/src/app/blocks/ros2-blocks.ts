import * as Blockly from 'blockly';

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

  
}