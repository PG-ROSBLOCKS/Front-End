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

export function create_server(code: string, name: string): string {
    const nameWithoutExtension = name.replace(/\.py$/, '');
    return `# Archivo ${nameWithoutExtension}.py generado por ROSBlocks
${getImports()}


class ${nameWithoutExtension.toUpperCase()}(Node):

    def __init__(self):
        super().__init__('${nameWithoutExtension.toLowerCase()}')
        # Registro del servidor en el grafo de ROS2.
        # Reemplaza SERVICE_TYPE por el tipo de servicio real (por ejemplo, AddTwoInts, Trigger, etc.)
        self.srv = self.create_service(SERVICE_TYPE, '${nameWithoutExtension.toLowerCase()}_service', self.callback)

    def callback(self, request, response):
        try:
            ${code}
            return response
        except Exception as e:
            self.get_logger().error(f"Error en callback: {e}")
            # Se envía una respuesta de error en caso de excepción
            response.error = str(e)
            return response


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
