export function sanitizePythonFilename(filename: string): string {
  let sanitized = filename
      .trim() // Removes spaces at the beginning and end
      .replace(/\s+/g, "_") // Replaces internal spaces with "_"
      .replace(/[^a-zA-Z0-9_]/g, "") // Removes disallowed characters
      .replace(/^(\d+)/, "_$1") // Prevents the filename from starting with a number
      .toLowerCase(); // Converts to lowercase for consistency

  // If it does not contain at least one letter, add the prefix "node_"
  if (!/[a-zA-Z]/.test(sanitized)) {
      sanitized = "node_" + sanitized;
  }

  return sanitized.concat(".py"); // Ensures that it ends with `.py`
}

export function extractFirstLine(text: string): { firstLine: string, remainingText: string } {
  // Split the text into lines
  const lines = text.split('\n');

  // Get the first line
  const firstLine = lines.shift() || ''; // shift() removes the first element

  // Rebuild the string without the first line
  const remainingText = lines.join('\n');

  return { firstLine, remainingText };
}

export function sanitizeSrvFilename(filename: string): string {
  let sanitized = filename
      .trim() // Removes spaces at the beginning and end
      .replace(/[^a-zA-Z0-9\s]/g, "") // Removes disallowed characters (except spaces)
      .replace(/\s+/g, "_") // Replaces spaces with "_"
      .replace(/_+/g, "") // Removes all "_" (as they are not valid)
      .replace(/^(\d+)/, "_$1"); // Prevents the filename from starting with a number

  // Ensure it starts with an uppercase letter (PascalCase)
  sanitized = sanitized.replace(/(^[a-z])/, (match) => match.toUpperCase());

  // If the result is empty, assign a generic name
  if (!sanitized) sanitized = "SrvUnnamed";

  return sanitized.concat(".srv"); // Ensures that it ends with `.srv`
}

export function sanitizeMsgFilename(filename: string): string {
  let sanitized = filename
      .trim() // Removes spaces at the beginning and end
      .replace(/[^a-zA-Z0-9\s]/g, "") // Removes disallowed characters (except spaces)
      .replace(/\s+/g, "_") // Replaces spaces with "_"
      .replace(/_+/g, "") // Removes all "_" (as they are not valid)
      .replace(/^(\d+)/, "_$1"); // Prevents the filename from starting with a number

  // Ensure it starts with an uppercase letter (PascalCase)
  sanitized = sanitized.replace(/(^[a-z])/, (match) => match.toUpperCase());

  // If the result is empty, assign a generic name
  if (!sanitized) sanitized = "MsgUnnamed";

  return sanitized.concat(".msg"); // Ensures that it ends with `.msg`
}

export function extractServiceFilename(input: string): string | null {
  const match = input.match(/^# Archivo (.+)\.srv generado por ROSBlocks/);
  return match ? match[1] : null; // Returns only the name
}

export function replaceServiceFilename(input: string, sanitizedName: string): string {
  return input.replace(/^# Archivo (.+\.srv) generado por ROSBlocks/, `# Archivo ${sanitizedName} generado por ROSBlocks`);
}

export function extractMessageFilename(input: string): string | null {
  const match = input.match(/^# Archivo (.+)\.msg generado por ROSBlocks/);
  return match ? match[1] : null; // Returns only the name
}

export function replaceMessageFilename(input: string, sanitizedName: string): string {
  return input.replace(/^# Archivo (.+\.msg) generado por ROSBlocks/, `# Archivo ${sanitizedName} generado por ROSBlocks`);
}

export function removeIndentation(code: string): string {
  return code
      .split('\n')
      .map(line => line.trimStart()) // Removes leading spaces and tabs from each line
      .join('\n');
}