"""One-second process accounting outside estimator/control callbacks."""
from pathlib import Path
import csv, json, os, signal, time

def main():
    run = Path(os.environ['DEMO_RUN_DIR'])
    stopped = False
    previous = {}
    ticks = os.sysconf('SC_CLK_TCK')
    pages = os.sysconf('SC_PAGE_SIZE')

    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with (run / 'process_resources.csv').open('x') as out:
        writer = csv.writer(out)
        writer.writerow(['wall_monotonic_s', 'role', 'pid', 'cpu_total_s', 'cpu_cores_interval', 'rss_mib'])
        while not stopped:
            now = time.monotonic()
            try:
                processes = json.loads((run / 'processes.json').read_text())
            except (OSError, ValueError):
                processes = []
            for process in processes:
                try:
                    data = Path('/proc', str(process['pid']), 'stat').read_text().rsplit(')', 1)[1].split()
                    cpu = (int(data[11]) + int(data[12])) / ticks
                    rss = int(data[21]) * pages / 1048576
                    old = previous.get(process['pid'])
                    usage = 0.0 if old is None else (cpu - old[1]) / max(0.001, now - old[0])
                    previous[process['pid']] = (now, cpu)
                    writer.writerow([now, process['role'], process['pid'], cpu, usage, rss])
                except (OSError, ValueError, IndexError):
                    pass
            out.flush()
            time.sleep(1.0)
if __name__ == '__main__':
    main()
