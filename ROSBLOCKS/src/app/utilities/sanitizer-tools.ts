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
      continue; // omitimos el marcador
    }

    if (!foundMarker) {
      header.push(line);
    } else {
      rest.push(line);
    }
  }

  if (!foundMarker) {
    return {
      headerText: '',
      codeText: lines.join('\n'), // devolver todo el código como está
    };
  }

  return {
    headerText: header.join('\n'),
    codeText: rest.join('\n'),
  };
}

export function indentSmartPreserveStructure(code: string, baseLevel = 2): string {
  const lines = code.split('\n').filter(line => line.trim() !== '');
  const result: string[] = [];
  const indent = (n: number) => TAB_SPACE.repeat(n);

  let currentIndent = baseLevel;

  for (const rawLine of lines) {
    const trimmed = rawLine.trim();

    // Detectar definiciones de funciones (def ...)
    const defMatch = /^def\s+\w+\(.*\):$/.exec(trimmed);
    if (defMatch) {
      // Se indenta a baseLevel - 1 como si estuviera dentro de una clase
      result.push(indent(baseLevel - 1) + trimmed);
      currentIndent = baseLevel;
      continue;
    }

    // Si se encuentra el marcador #main-sendrequest,
    // reiniciamos el nivel de indentación para que el código siguiente quede al nivel del main.
    if (trimmed === "#main-sendrequest") {
      currentIndent = baseLevel;
      result.push(indent(currentIndent) + trimmed);
      continue;
    }

    // Si la línea es un marcador de inicio de bloque (por ejemplo, #STARTREPEAT, #STARTWHILE, #STARTIF, #STARTELSE)
    if (/^#START/.test(trimmed)) {
      result.push(indent(currentIndent) + trimmed);
      currentIndent++; // Aumenta el nivel para el bloque que comienza
      continue;
    }

    // Si la línea es un marcador de fin de bloque (por ejemplo, #ENDREPEAT, #ENDWHILE, #ENDIF, #ENDELSE)
    if (/^#END/.test(trimmed)) {
      currentIndent = Math.max(currentIndent - 1, baseLevel);
      result.push(indent(currentIndent) + trimmed);
      continue;
    }

    // Para las demás líneas, se indenta con el nivel actual
    result.push(indent(currentIndent) + trimmed);
  }

  return result.join('\n');
}
export function removeOneIndentLevel(code: string): string {
  const lines = code.split('\n');
  const result = lines.map(line => {
    // Si la línea comienza con 4 espacios, los quitamos
    if (line.startsWith('    ')) {
      return line.slice(4);
    }
    return line;
  });
  return result.join('\n');
}


export function removeCommonIndentation(code: string) {
    let lines = code.split('\n');
  
    // Remove blank lines at the beginning and end
    while (lines.length && !lines[0].trim()) {
      lines.shift();
    }
    while (lines.length && !lines[lines.length - 1].trim()) {
      lines.pop();
    }
  
    // Calculates minimum indentation
    let minIndent = Infinity;
    for (const line of lines) {
      if (!line.trim()) continue; // Ignore empty lines
      const match = line.match(/^(\s*)/);
      const indentCount = match ? match[1].length : 0;
      if (indentCount < minIndent) {
        minIndent = indentCount;
      }
    }
    if (minIndent === Infinity) {
      return code; // If there are no lines with content
    }
  
    // Deletes minIndent on every line
    lines = lines.map(line => line.slice(minIndent));
    return lines.join('\n');
  }

  export function sanitizeGlobalVariables(code: string): string {
    const lines = code.split('\n');
  
    const varRegex = /^([a-zA-Z_][a-zA-Z0-9_]*)\s*=\s*None\s*$/;
    const classRegex = /^(\s*)class\s+\w+/;
    const defRegex = /^(\s*)def\s+\w+\(/;
  
    const globalVars = new Set<string>();
  
    // 1) Recolectar variables globales definidas como x = None a nivel toplevel
    for (const line of lines) {
      if (line.startsWith(' ') || line.startsWith('\t')) {
        continue;
      }
      const match = varRegex.exec(line.trim());
      if (match) {
        globalVars.add(match[1]);
      }
    }
  
    if (globalVars.size === 0) {
      return code;
    }
  
    const newLines: string[] = [];
    const classIndentStack: string[] = [];
  
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      newLines.push(line);
  
      // Detectar inicio de clase
      const classMatch = classRegex.exec(line);
      if (classMatch) {
        classIndentStack.push(classMatch[1]);
        continue;
      }
  
      // Limpiar clases cerradas por dedent
      while (
        classIndentStack.length > 0 &&
        line.trim() !== '' &&
        line.startsWith(classIndentStack[classIndentStack.length - 1]) === false
      ) {
        classIndentStack.pop();
      }
  
      // Si estamos dentro de una clase y encontramos un def, insertamos la línea global
      const defMatch = defRegex.exec(line);
      if (defMatch && classIndentStack.length > 0) {
        const indentForDef = defMatch[1];
        const indentForBody = indentForDef + TAB_SPACE;
        const joined = [...globalVars].sort().join(', ');
        newLines.push(`${indentForBody}global ${joined}`);
      }
    }
  
    return newLines.join('\n');
  }
  
  export function linesAfter(code: string): string {
    const marker = "#main-sendrequest";
    const index = code.indexOf(marker);
    if (index === -1) return "";
    // Extracts everything following the marker, without altering the original indentation.
    return removeOneIndentLevel(code.substring(index + marker.length))
  }

  export function linesBeforeComment(code: string): string {
    const marker = "#main-sendrequest";
    const index = code.indexOf(marker);
    if (index === -1) {
      return code.trimEnd();
    }
    return code.substring(0, index).trimEnd();
  }