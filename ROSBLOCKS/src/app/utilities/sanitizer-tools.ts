import * as Blockly from 'blockly/core';
import { pythonGenerator, Order} from 'blockly/python';
import { TAB_SPACE } from '../blocks/ros2-blocks-code';
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

export function indentSmart(code: string, defaultLevel: number = 2, structuredLevel: number = 1): string {
  if (!code.trim()) return '';

  const lines = code.split('\n');
  const firstLine = lines.find(line => line.trim() !== '') || '';

  // Detecta si el código comienza con una estructura como def, class, import, etc.
  const startsStructured = /^(def |class |#|import |from )/.test(firstLine.trim());

  const level = startsStructured ? structuredLevel : defaultLevel;
  const indent = TAB_SPACE.repeat(level);

  return lines
    .map(line => line ? indent + line : '')
    .join('\n');
}

export function indentSmartByLine(code: string, normalLevel = 2, structuredLevel = 1): string {
  if (!code.trim()) return '';

  const structuredRegex = /^\s*(def |class |import |from |#)/;
  const indentStructured = TAB_SPACE.repeat(structuredLevel);
  const indentNormal = TAB_SPACE.repeat(normalLevel);

  return code
    .split('\n')
    .map(line => {
      if (!line.trim()) return '';
      const indent = structuredRegex.test(line) ? indentStructured : indentNormal;
      return indent + line.trim();
    })
    .join('\n');
}

export function removeSelfInMain(code: string): string {
  const lines = code.split('\n');
  const result: string[] = [];

  let inMain = false;
  let mainIndent = '';

  for (const line of lines) {
    // Detectar inicio de main
    if (/^\s*def\s+main\s*\(/.test(line)) {
      inMain = true;
      const match = line.match(/^(\s*)def\s+main/);
      mainIndent = match ? match[1] + TAB_SPACE : TAB_SPACE; // Estimamos la indentación del bloque
      result.push(line);
      continue;
    }

    // Si estamos en main y la línea tiene indentación del bloque o más
    if (inMain) {
      const isIndented = line.startsWith(mainIndent) || line.trim() === '';

      if (isIndented) {
        // Solo si es línea no vacía
        if (line.trim() !== '') {
          result.push(line.replace(/self\./g, ''));
        } else {
          result.push(line); // líneas vacías se conservan
        }
      } else {
        // Salimos del bloque main
        inMain = false;
        result.push(line);
      }
    } else {
      result.push(line);
    }
  }

  return result.join('\n');
}

export function replaceSelfWithNodeInMain(code: string): string {
  const lines = code.split('\n');
  const result: string[] = [];

  let inMain = false;
  let mainIndent = '';

  for (const line of lines) {
    // Detecta el inicio de la función main
    if (/^\s*def\s+main\s*\(/.test(line)) {
      inMain = true;
      const match = line.match(/^(\s*)def\s+main/);
      mainIndent = match ? match[1] + TAB_SPACE : TAB_SPACE;
      result.push(line);
      continue;
    }

    if (inMain) {
      const isIndented = line.startsWith(mainIndent) || line.trim() === '';

      if (isIndented) {
        if (line.trim() !== '') {
          result.push(line.replace(/self\./g, 'node.'));
        } else {
          result.push(line);
        }
      } else {
        // Salimos del bloque main
        inMain = false;
        result.push(line);
      }
    } else {
      result.push(line);
    }
  }

  return result.join('\n');
}

export function reorderCodeBelowFirstMarker(code: string): string {
  const lines = code.split('\n');

  const markerRegex = /^(pub_sub|srv|msg|server\|.+|client\|.+)$/;

  const beforeMarker: string[] = [];
  const afterMarker: string[] = [];
  let markerLine: string | null = null;

  for (const rawLine of lines) {
    const line = rawLine.trim();
    if (!line) continue; // omitir líneas vacías

    if (!markerLine && markerRegex.test(line)) {
      markerLine = rawLine;
    } else if (!markerLine) {
      beforeMarker.push(rawLine);
    } else {
      afterMarker.push(rawLine);
    }
  }

  if (!markerLine) {
    // Si no se encuentra marcador, devolver todo sin modificar (sin líneas vacías)
    return [...beforeMarker, ...afterMarker].join('\n');
  }

  const result: string[] = [markerLine];

  if (beforeMarker.length > 0) {
    result.push(...beforeMarker, '# --- END OF BLOCKLY IMPORTS ---');
  }

  result.push(...afterMarker);

  return result.join('\n');
}




export function separateHeaderFromMarker(code: string): { headerText: string; codeText: string } {
  const lines = code.split('\n');
  const marker = '# --- END OF BLOCKLY IMPORTS ---';

  const header: string[] = [];
  const rest: string[] = [];

  let foundMarker = false;

  for (const line of lines) {
    const trimmed = line.trim();

    if (trimmed === marker.trim()) {
      foundMarker = true;
      continue; // omitimos el propio marcador
    }

    if (!foundMarker) {
      header.push(line);
    } else {
      rest.push(line);
    }
  }

  return {
    headerText: header.join('\n'),
    codeText: rest.join('\n'),
  };
}




