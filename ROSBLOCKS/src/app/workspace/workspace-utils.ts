export function parseMatrix(content: string): number[][] {
    return content.trim().split('\n').map(row =>
      row.trim().split('').map(char => (char === '1' ? 1 : 0))
    );
  }