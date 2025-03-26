import { pythonGenerator } from 'blockly/python';
import { getImports } from './ros2-blocks-code';
import { separateHeaderFromMarker } from '../utilities/sanitizer-tools';
export function create_publisher(code: string, name: string): string {
    const nameWithoutExtension = name.replace(/\.py$/, '');
    const { headerText, codeText: codeWithoutVariables } = separateHeaderFromMarker(code);
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}
${headerText}

class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
${codeWithoutVariables}


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
    const { headerText, codeText: codeWithoutVariables } = separateHeaderFromMarker(code);
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}
from sample_interfaces.srv import ${serverType}
${headerText}


class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
${codeWithoutVariables}


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

export function create_client(code: string, name: string, main_code: string, serverType: string): string {
    const nameWithoutExtension = name.replace(/\.py$/, '');
    const { headerText, codeText: codeWithoutVariables } = separateHeaderFromMarker(main_code);
    const TAB_SPACE = '    ';
    const { headerText: headerText2, codeText: codeWithoutVariables2 } = separateHeaderFromMarker(code);
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}
from sample_interfaces.srv import ${serverType}
${headerText}
${headerText2}

class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
${codeWithoutVariables2}

def main(args=None):
    rclpy.init(args=args)
    node = ${nameWithoutExtension.toUpperCase()}()
${codeWithoutVariables}

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()`;
}

export function create_map(code: string, name: string): string {
    const nameWithoutExtension = name.replace(/\.py$/, '');
    const TAB_SPACE = '    ';
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}
import ast

class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
${code}

def main(args=None):
    rclpy.init(args=args)

    node = ${nameWithoutExtension.toUpperCase()}()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`;
}
