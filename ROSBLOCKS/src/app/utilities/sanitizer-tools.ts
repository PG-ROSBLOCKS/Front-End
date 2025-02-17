export function sanitizePythonFilename(filename: string): string {
    return filename
      .trim() // Elimina espacios al inicio y al final
      .replace(/\s+/g, "_") // Reemplaza espacios internos con "_"
      .replace(/[^a-zA-Z0-9_]/g, "") // Elimina caracteres no permitidos
      .replace(/^(\d+)/, "_$1") // Evita que comience con un número
      .concat(".py") // Asegura que termine en `.py`
      .toLowerCase(); // Convierte a minúsculas para uniformidad
  }
  