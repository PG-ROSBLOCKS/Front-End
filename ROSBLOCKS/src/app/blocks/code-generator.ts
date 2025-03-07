import { getImports } from './ros2-blocks';
export function create_publisher(code: string, name: string): string {
    const nameWithoutExtension = name.replace(/\.py$/, '');
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}


class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
${code}


def main(args=None):
    rclpy.init(args=args)

    ${nameWithoutExtension.toLowerCase()} = ${nameWithoutExtension.toUpperCase()}()

    rclpy.spin(${nameWithoutExtension.toLowerCase()})    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ${nameWithoutExtension.toLowerCase()}.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()`;
}

export function create_server(code: string, name: string, serverType:string): string {
    const nameWithoutExtension = name.replace(/\.py$/, '');
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}
from sample_interfaces.srv import ${serverType}

class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
${code}


def main(args=None):
    rclpy.init(args=args)

    node = ${nameWithoutExtension.toUpperCase()}()

    rclpy.spin(node)    

    # Se destruye el nodo explícitamente (opcional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()`;
}
