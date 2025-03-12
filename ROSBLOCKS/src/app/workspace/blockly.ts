export let toolbox = {
    kind: 'categoryToolbox',
    contents: [
      {
        kind: 'category',
        name: 'Nodos',
        contents: [
          { kind: 'block', type: 'ros2_timer' },
          { kind: 'block', type: 'ros2_log' },
        ],
      },
      {
        kind: 'category',
        name: 'Tópicos',
        contents: [
          { kind: 'block', type: 'ros2_create_publisher' },
          { kind: 'block', type: 'ros2_minimal_publisher' },
          { kind: 'block', type: 'ros2_create_subscriber' },
          { kind: 'block', type: 'ros2_subscriber_msg_data' },
          { kind: 'block', type: 'ros2_turtlesim_pose_field' },
          { kind: 'block', type: 'ros2_print_msg_type' },
          { kind: 'block', type: 'ros2_publish_message' },
        ],
      },
      {
        kind: 'category',
        name: 'Servicios',
        contents: [
          { kind: 'block', type: 'ros2_service_block' },
          { kind: 'block', type: 'ros2_named_message' },
          { kind: 'block', type: 'ros_create_server' },
          { kind: 'block', type: 'ros2_service_available' },
        ],
      },
      {
        kind: 'category',
        name: 'Clientes',
        contents: [
          { kind: 'block', type: 'ros_create_client' },
          { kind: 'block', type: 'ros_send_request' },
        ],
      },
      {
        kind: 'category',
        name: 'Mensajes',
        contents: [
          { kind: 'block', type: 'ros2_message_block' },
          { kind: 'block', type: 'ros2_named_message' }
        ],
      },
      {
        kind: 'category',
        name: 'Condicionales',
        contents: [
          { kind: 'block', type: 'controls_if' },
          { kind: 'block', type: 'controls_ifelse' },
          { kind: 'block', type: 'logic_compare' },
          { kind: 'block', type: 'logic_operation' },
          { kind: 'block', type: 'logic_negate' },
          { kind: 'block', type: 'logic_boolean' },
          { kind: "block", type: "logic_ternary" },
        ],
      },
      {
        kind: 'category',
        name: 'Ciclos',
        contents: [
          { kind: "block", type: "controls_repeat" },
          { kind: "block", type: "controls_repeat_ext" },
          { kind: "block", type: "controls_whileUntil" },
          { kind: "block", type: "controls_for" },
          { kind: "block", type: "controls_forEach" },
        ],
      },
      {
        kind: 'category',
        name: 'Operaciones',
        contents: [
          { kind: "block", type: "math_number" },
          { kind: "block", type: "math_arithmetic" },
          { kind: "block", type: "math_single" },
          { kind: "block", type: "math_trig" },
          { kind: "block", type: "math_round" },
          { kind: "block", type: "math_random_int" },
          { kind: "block", type: "math_modulo" },
        ],
      },
      {
        kind: 'category',
        name: 'Variables',
        contents: [
          { kind: "block", type: "variables_get" },
          { kind: "block", type: "variables_set" },
        ],
      },
      {
        kind: 'category',
        name: 'Funciones',
        contents: [
          { kind: "block", type: "procedures_defnoreturn" },
          { kind: "block", type: "procedures_defreturn" },
          { kind: "block", type: "procedures_callnoreturn" },
          { kind: "block", type: "procedures_callreturn" },
          { kind: "block", type: "procedures_ifreturn" },
        ],
      },
      {
        kind: 'category',
        name: 'Texto',
        contents: [
          { kind: "block", type: "text" },
          { kind: "block", type: "text_join" },
          { kind: "block", type: "text_append" },
          { kind: "block", type: "text_length" },
          { kind: "block", type: "text_isEmpty" },
          { kind: "block", type: "text_indexOf" },
          { kind: "block", type: "text_charAt" },
          { kind: "block", type: "text_getSubstring" },
          { kind: "block", type: "text_changeCase" },
          { kind: "block", type: "text_trim" },
          { kind: "block", type: "text_print" },
        ]
      },
      {
        kind: 'category',
        name: 'Turtlesim',
        contents: [
          { kind: "block", type: "ros2_publish_twist" },
          { kind: "block", type: "ros2_rotate_turtle" },
          { kind: "block", type: "ros2_turtle_set_pose" },
          { kind: "block", type: "ros2_sleep" },
          { kind: "block", type: "ros2_kill_turtle" },
          { kind: "block", type: "ros2_spawn_turtle" },
          { kind: "block", type: "ros2_turtle_pen_up" },
          { kind: "block", type: "ros2_turtle_pen_down" },
          { kind: "block", type: "ros2_turtle_set_pen" },
        ]
      }
    ],
  };