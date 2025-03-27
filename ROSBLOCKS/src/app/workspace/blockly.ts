import { blockColors } from "../blocks/color-palette";

export let toolbox = {
  kind: 'categoryToolbox',
  contents: [
    {
      kind: 'category',
      name: 'ROS2 Blocks',
      contents: [
        {
          kind: 'category',
          name: 'Turtlesim',
          colour: blockColors.Turtlesim,
          contents: [
            { kind: "block", type: "ros2_turtlesim_publisher" },
            { kind: "block", type: "ros2_publish_twist" },
            { kind: "block", type: "ros2_turtle_set_pose" },
            { kind: "block", type: "ros2_kill_turtle" },
            { kind: "block", type: "ros2_spawn_turtle" },
            { kind: "block", type: "ros2_turtle_set_pen" },
            { kind: "block", type: "ros2_turtle_rotate" },
          ]
        },
        {
        kind: 'category',
        name: 'Nodes',
        colour: blockColors.Nodes,
        contents: [
          { kind: 'block', type: 'ros2_timer' },
          { kind: 'block', type: 'ros2_log' },
        ],
      },
      {
        kind: 'category',
        name: 'Topics',
        colour: blockColors.Topics,
        contents: [
          { kind: 'block', type: 'ros2_create_publisher' },
          { kind: 'block', type: 'ros2_create_subscriber' },
          { kind: 'block', type: 'ros2_print_msg_type' },
          { kind: 'block', type: 'ros2_publish_message' },
        ],
      },
      {
        kind: 'category',
        name: 'Services',
        colour: blockColors.Services,
        contents: [
          { kind: 'block', type: 'ros2_service_block' },
          { kind: 'block', type: 'ros_create_server' },
        ],
      },
      {
        kind: 'category',
        name: 'Clients',
        colour: blockColors.Clients,
        contents: [
          { kind: 'block', type: 'ros_create_client' },
          { kind: 'block', type: 'ros_send_request' },
        ],
      },
      {
        kind: 'category',
        name: 'Messages',
        colour: blockColors.Messages,
        contents: [
          { kind: 'block', type: 'ros2_message_block' },
        ],
      },
      {
        kind: 'category',
        name: 'Variables',
        contents: [
          { kind: 'block', type: 'ros2_subscriber_msg_data' },
          { kind: 'block', type: 'srv_response_set_field' },
        ],
      }]
    },
    {
      kind: 'category',
      name: 'Programming Blocks',
      contents: [
        {
          kind: 'category',
          name: 'Conditionals',
          colour: blockColors.Conditionals,
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
          name: 'Cycles',
          colour: '#3E7E7E',
          contents: [
            { kind: "block", type: "controls_repeat" },
            { kind: "block", type: "controls_repeat_ext" },
            { kind: "block", type: "controls_whileUntil" },
            { kind: "block", type: "controls_for" },
            { kind: "block", type: "controls_forEach" },
            { kind: "block", type: "ros2_sleep" },
          ],
        },
        {
          kind: 'category',
          name: 'Operations',
          colour: '#569BBD',
          contents: [
            { kind: "block", type: "math_number" },
            { kind: "block", type: "float_number" },
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
          colour: '#3D65A8',
          contents: [
            { kind: "block", type: "variables_get" },
            { kind: "block", type: "variables_set" },
            { kind: 'block', type: 'ros2_named_message' }
          ],
        },
        {
          kind: 'category',
          name: 'Functions',
          colour: '#897099',
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
          name: 'Text',
          colour: '#6835BB',
          contents: [
            { kind: "block", type: "text" },
            { kind: "block", type: "text_char_to_ascii" },
            { kind: "block", type: "text_ascii_to_char" },
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
      ]
    }
  ]
};

export function updateDynamicCategoryInToolbox(
  toolboxObj: any,
  parentCategoryName: string,
  subCategoryName: string,
  dynamicCategoryName: string,
  newDynamicCategory: any
): void {
  const contents = toolboxObj.contents;
  const parentCategoryIdx = contents.findIndex((c: any) => c.name === parentCategoryName);
  if (parentCategoryIdx !== -1) {
    const parentCategory = contents[parentCategoryIdx];
    const subCategoryIdx = parentCategory.contents.findIndex((c: any) => c.name === subCategoryName);
    if (subCategoryIdx !== -1) {
      const subCategory = parentCategory.contents[subCategoryIdx];
      const dynamicCategoryIdx = subCategory.contents.findIndex((c: any) => c.name === dynamicCategoryName);
      if (dynamicCategoryIdx !== -1) {
        // Si ya existe, se actualiza
        subCategory.contents[dynamicCategoryIdx] = newDynamicCategory;
      } else {
        // Si no existe, se agrega
        subCategory.contents.push(newDynamicCategory);
      }
    } else {
      console.warn(`Subcategoría "${subCategoryName}" no encontrada dentro de "${parentCategoryName}".`);
    }
  } else {
    console.warn(`Categoría principal "${parentCategoryName}" no encontrada en la toolbox.`);
  }
}