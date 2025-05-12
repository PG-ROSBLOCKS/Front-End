/* perf-utils.ts */
export class PerfTest {
  constructor(private id: string) {}

  private pref(label: string): string {
    return `${this.id}:${label}`;
  }

  /** Place a mark called "<id>:<label>" */
  mark(label: string): void {
    performance.mark(this.pref(label));
  }

  /** Measure "<id>:<name>" once between two marks of this test */
  measure(name: string, start: string, end: string): void {
    const full = this.pref(name);
    if (!performance.getEntriesByName(full, 'measure').length) {
      performance.measure(full, this.pref(start), this.pref(end));
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

