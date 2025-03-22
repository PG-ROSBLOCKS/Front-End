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
  

export function paintMap(matrix: any[][]): string {
    if (!isValidMap(matrix)) {
        return '';
    }

    return `import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn, Kill
import time
import ast
import threading

class MatrixPainter(Node):
    def __init__(self):
        super().__init__('matrix_painter')
        
        # Parámetro con la matriz en forma de string
        self.declare_parameter('matrix_str', '${MatrixToString(matrix)}')
        
        self.cli_tp = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli_tp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /turtle1/teleport_absolute...')
        while not self.cli_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /turtle1/set_pen...')
        
        # Cliente para el servicio spawn que creará nuevas tortugas
        self.cli_spawn = self.create_client(Spawn, '/spawn')
        while not self.cli_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /spawn service...')
        
        self.paint_matrix()

    def set_pen_turtle(self, turtle_name, r, g, b, width, off):
        client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Esperando /{turtle_name}/set_pen...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off  # 0 = dibuja, 1 = no dibuja
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def teleport_turtle(self, turtle_name, x, y, theta=0.0):
        client = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Esperando /{turtle_name}/teleport_absolute...')
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.05)

    def kill_turtle(self, turtle_name):
        client = self.create_client(Kill, '/kill')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /kill service...')
        req = Kill.Request()
        req.name = turtle_name
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'{turtle_name} ha sido eliminada.')

    def paint_cell(self, turtle_name, x_start, y_start, cell_width, cell_height, pencil_size):
        # Configuramos la tortuga: desactivamos el lápiz, la posicionamos y activamos el lápiz
        self.set_pen_turtle(turtle_name, 0, 0, 0, 0, 1)
        self.teleport_turtle(turtle_name, x_start, y_start)
        self.set_pen_turtle(turtle_name, 0, 0, 0, pencil_size, 0)

        # Dibujar el cuadrado recorriendo sus cuatro vértices:
        # Lado superior
        self.teleport_turtle(turtle_name, x_start + cell_width, y_start)
        # Lado derecho
        self.teleport_turtle(turtle_name, x_start + cell_width, y_start - cell_height)
        # Lado inferior
        self.teleport_turtle(turtle_name, x_start, y_start - cell_height)
        # Cierre del cuadrado
        self.teleport_turtle(turtle_name, x_start, y_start)

        # Una vez terminado el dibujo, eliminamos la tortuga para liberar recursos
        self.kill_turtle(turtle_name)

    def paint_matrix(self):
        matrix_str = self.get_parameter('matrix_str').get_parameter_value().string_value
        matrix = ast.literal_eval(matrix_str)  # Convertir string a lista de listas
        
        # Definición del área a pintar
        x0 = 0
        y0 = 10.6
        xf = 11
        yf = 0

        size = len(matrix)
        cell_width = (xf - x0) / size
        cell_height = (y0 - yf) / size
        pencil_size = 6

        threads = []
        # Por cada celda con valor 1 se spawnea una tortuga y se crea un hilo para dibujar el cuadrado
        for i in range(size):
            for j in range(size):
                if matrix[i][j] == 1:
                    x_start = x0 + j * cell_width
                    y_start = y0 - i * cell_height
                    
                    # Request para spawnear la tortuga en la posición de inicio de la celda
                    spawn_req = Spawn.Request()
                    spawn_req.x = x_start
                    spawn_req.y = y_start
                    spawn_req.theta = 0.0
                    spawn_req.name = f'turtle_{i}_{j}'  # Nombre único

                    future_spawn = self.cli_spawn.call_async(spawn_req)
                    rclpy.spin_until_future_complete(self, future_spawn)
                    turtle_name = future_spawn.result().name
                    self.get_logger().info(f'Spawneada {turtle_name} en ({x_start:.2f}, {y_start:.2f})')

                    # Se crea un hilo para que esta tortuga dibuje el cuadrado y luego se elimine
                    thread = threading.Thread(target=self.paint_cell, args=(turtle_name, x_start, y_start, cell_width, cell_height, pencil_size))
                    thread.start()
                    threads.append(thread)
        
        # Esperamos a que todas las tortugas terminen de pintar y se eliminen
        for t in threads:
            t.join()

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MatrixPainter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

`;
}