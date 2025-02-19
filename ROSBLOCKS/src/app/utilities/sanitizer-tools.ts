export function sanitizePythonFilename(filename: string): string {
  let sanitized = filename
      .trim() // Elimina espacios al inicio y al final
      .replace(/\s+/g, "_") // Reemplaza espacios internos con "_"
      .replace(/[^a-zA-Z0-9_]/g, "") // Elimina caracteres no permitidos
      .replace(/^(\d+)/, "_$1") // Evita que comience con un número
      .toLowerCase(); // Convierte a minúsculas para uniformidad

  // Si no contiene al menos una letra, agregar un prefijo "node_"
  if (!/[a-zA-Z]/.test(sanitized)) {
      sanitized = "node_" + sanitized;
  }

  return sanitized.concat(".py"); // Asegura que termine en `.py`
}
