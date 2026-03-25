import statistics
import time
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tools.walk import DEFAULT_ZERO, GaitParams, generate_frame


def run_benchmark(iterations: int = 5000):
    params = GaitParams()
    zero = DEFAULT_ZERO.copy()
    durations_ms = []
    t = 0.0
    for _ in range(iterations):
        start = time.perf_counter()
        generate_frame(t, zero, params)
        durations_ms.append((time.perf_counter() - start) * 1000.0)
        t += params.dt
    total_ms = sum(durations_ms)
    fps = iterations / (total_ms / 1000.0)
    p50 = statistics.median(durations_ms)
    p95 = statistics.quantiles(durations_ms, n=100)[94]
    p99 = statistics.quantiles(durations_ms, n=100)[98]
    print(f"iterations={iterations}")
    print(f"avg_ms={total_ms / iterations:.4f}")
    print(f"p50_ms={p50:.4f}")
    print(f"p95_ms={p95:.4f}")
    print(f"p99_ms={p99:.4f}")
    print(f"fps={fps:.2f}")


if __name__ == "__main__":
    run_benchmark()
