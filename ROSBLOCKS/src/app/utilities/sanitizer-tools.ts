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

export function extractFirstLine(text: string): { firstLine: string, remainingText: string } {
  // Dividir el texto en líneas
  const lines = text.split('\n');

  // Obtener la primera línea
  const firstLine = lines.shift() || ''; // shift() saca el primer elemento

  // Reconstruir el string sin la primera línea
  const remainingText = lines.join('\n');

  return { firstLine, remainingText };
}

export function sanitizeSrvFilename(filename: string): string {
  let sanitized = filename
      .trim() // Elimina espacios al inicio y al final
      .replace(/[^a-zA-Z0-9\s]/g, "") // Elimina caracteres no permitidos (excepto espacios)
      .replace(/\s+/g, "_") // Reemplaza espacios por "_"
      .replace(/_+/g, "") // Elimina todos los "_" (porque no son válidos)
      .replace(/^(\d+)/, "_$1"); // Evita que comience con un número

  // Asegurar que comience con una letra mayúscula (PascalCase)
  sanitized = sanitized.replace(/(^[a-z])/, (match) => match.toUpperCase());

  // Si el resultado está vacío, asignar un nombre genérico
  if (!sanitized) sanitized = "SrvUnnamed";

  return sanitized.concat(".srv"); // Asegura que termine en `.srv`
}

export function sanitizeMsgFilename(filename: string): string {
  let sanitized = filename
      .trim() // Elimina espacios al inicio y al final
      .replace(/[^a-zA-Z0-9\s]/g, "") // Elimina caracteres no permitidos (excepto espacios)
      .replace(/\s+/g, "_") // Reemplaza espacios por "_"
      .replace(/_+/g, "") // Elimina todos los "_" (porque no son válidos)
      .replace(/^(\d+)/, "_$1"); // Evita que comience con un número

  // Asegurar que comience con una letra mayúscula (PascalCase)
  sanitized = sanitized.replace(/(^[a-z])/, (match) => match.toUpperCase());

  // Si el resultado está vacío, asignar un nombre genérico
  if (!sanitized) sanitized = "MsgUnnamed";

  return sanitized.concat(".msg"); // Asegura que termine en `.msg`
}

export function extractServiceFilename(input: string): string | null {
  const match = input.match(/^# Archivo (.+)\.srv generado por ROSBlocks/);
  return match ? match[1] : null; // Retorna solo el nombre
}

export function replaceServiceFilename(input: string, sanitizedName: string): string {
  return input.replace(/^# Archivo (.+\.srv) generado por ROSBlocks/, `# Archivo ${sanitizedName} generado por ROSBlocks`);
}

export function extractMessageFilename(input: string): string | null {
  const match = input.match(/^# Archivo (.+)\.msg generado por ROSBlocks/);
  return match ? match[1] : null; // Retorna solo el nombre
}

export function replaceMessageFilename(input: string, sanitizedName: string): string {
  return input.replace(/^# Archivo (.+\.msg) generado por ROSBlocks/, `# Archivo ${sanitizedName} generado por ROSBlocks`);
}

export function removeIndentation(code: string): string {
  return code
      .split('\n')
      .map(line => line.trimStart()) // 🔹 Elimina espacios y tabulaciones al inicio de cada línea
      .join('\n');
}


