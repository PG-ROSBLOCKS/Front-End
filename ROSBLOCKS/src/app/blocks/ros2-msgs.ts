import { customMsgList, msgList, MsgVariable } from "../shared/msg-list";

export const common_msgs: [string, string][] = [
  ['String (std_msgs)', 'std_msgs.msg.String'],
  ['Bool (std_msgs)', 'std_msgs.msg.Bool'],
  ['Int64 (std_msgs)', 'std_msgs.msg.Int64'],
  ['Char (std_msgs)', 'std_msgs.msg.Char'],
  ['Float32 (std_msgs)', 'std_msgs.msg.Float32'],
  ['Twist (geometry_msgs)', 'geometry_msgs.msg.Twist'],
  ['Odometry (nav_msgs)', 'nav_msgs.msg.Odometry'],
  ['Pose (turtlesim)', 'turtlesim.msg.Pose'],

  ['Int16 (std_msgs)', 'std_msgs.msg.Int16'],
  ['Empty (std_msgs)', 'std_msgs.msg.Empty'],
  ['Float64 (std_msgs)', 'std_msgs.msg.Float64'],
  ['ColorRGBA (std_msgs)', 'std_msgs.msg.ColorRGBA'],
  ['Header (std_msgs)', 'std_msgs.msg.Header'],
  ['Byte (std_msgs)', 'std_msgs.msg.Byte'],

  //Geometry
  ['Pose (geometry_msgs)', 'geometry_msgs.msg.Pose'],
  ['Vector3 (geometry_msgs)', 'geometry_msgs.msg.Vector3'],
  ['Point (geometry_msgs)', 'geometry_msgs.msg.Point'],
  ['Quaternion (geometry_msgs)', 'geometry_msgs.msg.Quaternion'],
  ['Transform (geometry_msgs)', 'geometry_msgs.msg.Transform'],
  ['Pose2D (geometry_msgs)', 'geometry_msgs.msg.Pose2D'],
  ['Wrench (geometry_msgs)', 'geometry_msgs.msg.Wrench']
];
export const common_msgs_for_custom: [string, string][] = [
  ['String (std_msgs)', 'string'],
  ['Bool (std_msgs)', 'bool'],
  ['Int64 (std_msgs)', 'int64'],
  ['Char (std_msgs)', 'char'],
  ['Float32 (std_msgs)', 'float32'],
  ['Twist (geometry_msgs)', 'geometry_msgs/Twist'],
  ['Odometry (nav_msgs)', 'nav_msgs/Odometry'],
  ['Pose (turtlesim)', 'turtlesim/Pose']
];

const common_msg_fields: Record<string, MsgVariable[]> = {
    'std_msgs.msg.String': [{ name: 'data', type: 'string' }],
    'std_msgs.msg.Bool': [{ name: 'data', type: 'bool' }],
    'std_msgs.msg.Int64': [{ name: 'data', type: 'int64' }],
    'std_msgs.msg.Char': [{ name: 'data', type: 'char' }],
    'std_msgs.msg.Float32': [{ name: 'data', type: 'float32' }],
    'std_msgs.msg.Int16': [{ name: 'data', type: 'int16' }],
    'std_msgs.msg.Float64': [{ name: 'data', type: 'float64' }],
    'std_msgs.msg.Empty': [],
    'std_msgs.msg.ColorRGBA': [
        { name: 'r', type: 'float32' },
        { name: 'g', type: 'float32' },
        { name: 'b', type: 'float32' },
        { name: 'a', type: 'float32' }
    ],
    'std_msgs.msg.Header': [
        { name: 'stamp', type: 'builtin_interfaces/Time' },
        { name: 'frame_id', type: 'string' }
    ],
    'std_msgs.msg.Byte': [{ name: 'data', type: 'byte' }],

    'geometry_msgs.msg.Point': [
        { name: 'x', type: 'float64' },
        { name: 'y', type: 'float64' },
        { name: 'z', type: 'float64' }
    ],
    'geometry_msgs.msg.Quaternion': [
        { name: 'x', type: 'float64' },
        { name: 'y', type: 'float64' },
        { name: 'z', type: 'float64' },
        { name: 'w', type: 'float64' }
    ],
    'geometry_msgs.msg.Vector3': [
        { name: 'x', type: 'float64' },
        { name: 'y', type: 'float64' },
        { name: 'z', type: 'float64' }
    ],
    'geometry_msgs.msg.Pose': [
        { name: 'position', type: 'geometry_msgs/Point' },
        { name: 'orientation', type: 'geometry_msgs/Quaternion' }
    ],
    'geometry_msgs.msg.Pose2D': [
        { name: 'x', type: 'float64' },
        { name: 'y', type: 'float64' },
        { name: 'theta', type: 'float64' }
    ],
    'geometry_msgs.msg.Twist': [
        { name: 'linear', type: 'geometry_msgs/Vector3' },
        { name: 'angular', type: 'geometry_msgs/Vector3' }
    ],
    'geometry_msgs.msg.Transform': [
        { name: 'translation', type: 'geometry_msgs/Vector3' },
        { name: 'rotation', type: 'geometry_msgs/Quaternion' }
    ],
    'geometry_msgs.msg.Wrench': [
        { name: 'force', type: 'geometry_msgs/Vector3' },
        { name: 'torque', type: 'geometry_msgs/Vector3' }
    ],

    'nav_msgs.msg.Odometry': [
        { name: 'header', type: 'std_msgs/Header' },
        { name: 'child_frame_id', type: 'string' },
        { name: 'pose', type: 'geometry_msgs/PoseWithCovariance' },
        { name: 'twist', type: 'geometry_msgs/TwistWithCovariance' }
    ],

    // Campos mínimos para tipos usados en Odometry
    'geometry_msgs.msg.PoseWithCovariance': [
        { name: 'pose', type: 'geometry_msgs/Pose' },
        { name: 'covariance', type: 'float64[36]' }
    ],
    'geometry_msgs.msg.TwistWithCovariance': [
        { name: 'twist', type: 'geometry_msgs/Twist' },
        { name: 'covariance', type: 'float64[36]' }
    ],

    'turtlesim.msg.Pose': [
        { name: 'x', type: 'float32' },
        { name: 'y', type: 'float32' },
        { name: 'theta', type: 'float32' },
        { name: 'linear_velocity', type: 'float32' },
        { name: 'angular_velocity', type: 'float32' }
    ],

    // Extra: tipo auxiliar usado por std_msgs/Header
    'builtin_interfaces.msg.Time': [
        { name: 'sec', type: 'int32' },
        { name: 'nanosec', type: 'uint32' }
    ]
};
export function initializeCommonMsgs(): void {
    for (const [label, name] of common_msgs) {
        if (!msgList.find(m => m.name === name)) {
        const fields = common_msg_fields[name] || [];
        msgList.push({ name, fields });
        }
    }
    for (const [label, name] of common_msgs) {
        if (!customMsgList.find(m => m.name === name)) {
        const fields = common_msg_fields[name] || [];
        customMsgList.push({ name, fields });
        }
    }
}
  