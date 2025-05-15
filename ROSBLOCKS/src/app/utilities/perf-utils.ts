/* perf-utils.ts */
export class PerfTest {
  constructor(private id: string) {}

  private pref(label: string): string {
    return `${this.id}:${label}`;
  }

  /** Place a mark called "<id>:<label>" safely */
  mark(label: string): void {
    const markName = this.pref(label);
    try {
      performance.mark(markName);
    } catch {
      // ignore if mark fails (e.g., invalid or duplicates)
    }
  }

  /** Measure "<id>:<name>" once between two marks of this test safely */
  measure(name: string, start: string, end: string): void {
    const measureName = this.pref(name);
    const startMark = this.pref(start);
    const endMark = this.pref(end);

    // Only measure if not already done and both marks exist
    const hasMeasure = performance.getEntriesByName(measureName, 'measure').length > 0;
    const hasStart = performance.getEntriesByName(startMark, 'mark').length > 0;
    const hasEnd = performance.getEntriesByName(endMark, 'mark').length > 0;

    if (!hasMeasure && hasStart && hasEnd) {
      try {
        performance.measure(measureName, startMark, endMark);
      } catch {
        // ignore measurement errors
      }
    }
  }

  /** Only this test’s measures */
  getMeasures(): PerformanceMeasure[] {
    return performance
      .getEntriesByType('measure')
      .filter(e => e.name.startsWith(`${this.id}:`)) as PerformanceMeasure[];
  }

  /** Clear only this test’s marks & measures */
  clear(): void {
    const prefix = `${this.id}:`;
    for (const e of performance.getEntries()) {
      if (!e.name.startsWith(prefix)) continue;
      if (e.entryType === 'mark')    performance.clearMarks(e.name);
      else if (e.entryType === 'measure') performance.clearMeasures(e.name);
    }
  }
}

export function printSingle(perfId: string, measureName: string) {
  const entry = performance.getEntriesByName(`${perfId}:${measureName}`, 'measure')[0] as PerformanceMeasure;
  if (entry) {
    console.table([
      { test: measureName, duration: `${entry.duration.toFixed(2)} ms` }
    ]);
  }
}

export function printAllPlay(globalPerf: PerfTest | null) {
  if (!globalPerf) return;
  const m = globalPerf.getMeasures().find(e => e.name.endsWith(':play_all'));
  if (m) {
    console.table([{ test: 'play_all', duration: `${m.duration.toFixed(2)} ms` }]);
  }
}

export const globalMonitorPerf = new PerfTest('global');
