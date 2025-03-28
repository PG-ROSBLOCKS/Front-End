import { create_map } from "../blocks/code-generator";
import { addImport, getImports } from "../blocks/ros2-blocks-code";

export function isValidMap(matrix: any[][]): boolean {
    const numFilas = matrix.length;
  
    if (numFilas === 0) return false;
  
    for (const fila of matrix) {
        if (!Array.isArray(fila) || fila.length !== numFilas) {
            return false;
        }

        for (const valor of fila) {
            if (valor !== 0 && valor !== 1) {
                return false;
            }
        }
    }
    return true;
}

function MatrixToString(matrix: number[][]): String {
    return '[' +
      matrix
        .map(fila => '  [' + fila.join(', ') + ']') + ']';
  }

function generarPalabra(): string {
    const letras = 'abcdefghijklmnopqrstuvwxyz';
    let palabra = '';

    for (let i = 0; i < 5; i++) {
        const indice = Math.floor(Math.random() * letras.length);
        palabra += letras[indice];
    }

    return palabra;
}

export function paintMap(matrix: any[][]): string {
    if (!isValidMap(matrix)) {
        console.log("uuuuppps");
        
        return '';
    }

    let turtle = generarPalabra()

    addImport('turtlesim.srv.TeleportAbsolute');
    addImport('turtlesim.srv.SetPen');
    addImport('turtlesim.srv.Spawn');
    addImport('turtlesim.srv.Kill');

    getImports()

    const codeBody = `
        self.matrix = []
        self.history = {} 

    def run(self):
        self.declare_parameter('matrix_str', '${MatrixToString(matrix)}')
        self.turtle_name = '${turtle}'
        self.spawn_turtle()

        self.cli_tp = self.create_client(TeleportAbsolute, f'/{self.turtle_name}/teleport_absolute')
        self.cli_pen = self.create_client(SetPen, f'/{self.turtle_name}/set_pen')

        self.get_logger().info('Esperando servicios...')
        self.cli_tp.wait_for_service()
        self.cli_pen.wait_for_service()
        self.get_logger().info('Servicios disponibles. Dibujando...')

        self.draw_matrix()
        self.kill_turtle()
        self.get_logger().info('Dibujo terminado. Iniciando monitoreo...')
        self.monitor_alive_turtles()

    def monitor_alive_turtles(self):
        poses = {}
        subs = {}
        avisados = set()
        visitados = {}  # Últimas posiciones válidas (fuera de celdas marcadas)

        x0, y0 = 0.0, 10.6
        xf, yf = 11.0, 0.0
        size = len(self.matrix)
        cell_w = (xf - x0) / size
        cell_h = (y0 - yf) / size

        x_centro = 5.5
        y_centro = 5.5

        self.get_logger().info('Monitoreando posiciones de tortugas...')

        last_print_time = time.time()
        last_subs_print_time = time.time()
        subs_print_interval = 5

        while rclpy.ok():
            topics = self.get_topic_names_and_types()
            turtles_alive = set()

            for name, _ in topics:
                parts = name.strip('/').split('/')
                if len(parts) >= 2 and parts[1] == 'pose':
                    turtle_name = parts[0]
                    topic_name = f'/{turtle_name}/pose'
                    publishers = self.get_publishers_info_by_topic(topic_name)
                    if publishers:
                        turtles_alive.add(turtle_name)

            def make_callback(turtle):
                def callback(msg):
                    poses[turtle] = (msg.x, msg.y, msg.theta)
                return callback

            for turtle in turtles_alive:
                if turtle not in subs:
                    topic = f'/{turtle}/pose'
                    subs[turtle] = self.create_subscription(Pose, topic, make_callback(turtle), 10)
                    self.get_logger().info(f'Suscripción creada para: {turtle}')

            turtles_to_remove = [t for t in subs if t not in turtles_alive]
            for turtle in turtles_to_remove:
                self.destroy_subscription(subs[turtle])
                del subs[turtle]
                poses.pop(turtle, None)
                avisados.discard(turtle)
                visitados.pop(turtle, None)
                self.get_logger().info(f'Suscripción eliminada para: {turtle}')

            rclpy.spin_once(self, timeout_sec=0.01)  # 🔁 Revisión más frecuente

            current_time = time.time()
            if current_time - last_print_time >= 0.1:  # 📡 Actualización más frecuente
                for turtle in sorted(subs.keys()):
                    if turtle in poses:
                        x, y, theta = poses[turtle]
                        self.get_logger().info(f'{turtle} -> x: {x:.2f}, y: {y:.2f}, θ: {theta:.2f}')

                        col = int((x - x0) / cell_w)
                        row = int((y0 - y) / cell_h)

                        if 0 <= row < size and 0 <= col < size:
                            if self.matrix[row][col] == 1:
                                self.get_logger().info(f'✅ Tortuga "{turtle}" está DENTRO de una celda marcada ({row}, {col})')

                                if turtle in visitados:
                                    x_prev, y_prev, theta_prev = visitados[turtle]
                                    self.get_logger().warn(f'🔄 Teletransportando a {turtle} a posición segura ({x_prev:.2f}, {y_prev:.2f})')
                                else:
                                    x_prev, y_prev, theta_prev = x_centro, y_centro, 0.0
                                    self.get_logger().warn(f'🛑 Tortuga "{turtle}" sin posición válida previa. Enviando al centro ({x_centro}, {y_centro})')

                                tp_client = self.create_client(TeleportAbsolute, f'/{turtle}/teleport_absolute')
                                tp_client.wait_for_service()
                                req = TeleportAbsolute.Request()
                                req.x = x_prev
                                req.y = y_prev
                                req.theta = theta_prev
                                future = tp_client.call_async(req)
                                rclpy.spin_until_future_complete(self, future)

                        else:
                            self.get_logger().warn(f'⚠️ Tortuga "{turtle}" está fuera del área del mapa')
                    elif turtle not in avisados:
                        self.get_logger().warn(f'No se pudo obtener posición de {turtle} (aún)')
                        avisados.add(turtle)

                last_print_time = current_time

            if current_time - last_subs_print_time >= subs_print_interval:
                self.get_logger().info(f'Suscripciones activas: {sorted(list(subs.keys()))}')
                last_subs_print_time = current_time

    def spawn_turtle(self):
        cli_spawn = self.create_client(Spawn, '/spawn')
        cli_spawn.wait_for_service()
        req = Spawn.Request()
        req.x = 5.0
        req.y = 5.0
        req.theta = 0.0
        req.name = self.turtle_name
        future = cli_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Tortuga {future.result().name} creada.')
        else:
            self.get_logger().error('Fallo al crear turtle')

    def kill_turtle(self):
        cli_kill = self.create_client(Kill, '/kill')
        cli_kill.wait_for_service()
        req = Kill.Request()
        req.name = self.turtle_name
        future = cli_kill.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Tortuga {self.turtle_name} eliminada.')

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.cli_pen.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def teleport(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self.cli_tp.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def draw_matrix(self):
        matrix_str = self.get_parameter('matrix_str').get_parameter_value().string_value
        matrix = ast.literal_eval(matrix_str)
        self.matrix = matrix  # Guardar para uso en monitoreo

        x0, y0 = 0.0, 10.6
        xf, yf = 11.0, 0.0

        size = len(matrix)
        cell_w = (xf - x0) / size
        cell_h = (y0 - yf) / size
        pen_size = 6

        self.set_pen(0, 0, 0, pen_size, 1)  # Apagar lápiz

        for i in range(1, size):
            j = 0
            while j < size:
                if matrix[i][j] != matrix[i - 1][j]:
                    j_start = j
                    while j < size and matrix[i][j] != matrix[i - 1][j]:
                        j += 1
                    x1 = x0 + j_start * cell_w
                    x2 = x0 + j * cell_w
                    y = y0 - i * cell_h
                    self.teleport(x1, y)
                    self.set_pen(0, 0, 0, pen_size, 0)
                    self.teleport(x2, y)
                    self.set_pen(0, 0, 0, pen_size, 1)
                else:
                    j += 1

        for j in range(1, size):
            i = 0
            while i < size:
                if matrix[i][j] != matrix[i][j - 1]:
                    i_start = i
                    while i < size and matrix[i][j] != matrix[i][j - 1]:
                        i += 1
                    x = x0 + j * cell_w
                    y1 = y0 - i_start * cell_h
                    y2 = y0 - i * cell_h
                    self.teleport(x, y1)
                    self.set_pen(0, 0, 0, pen_size, 0)
                    self.teleport(x, y2)
                    self.set_pen(0, 0, 0, pen_size, 1)
                else:
                    i += 1

`
    const pythonCode = create_map(codeBody, 'paint_map');

    return pythonCode;
}