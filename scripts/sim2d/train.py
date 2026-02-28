#!/usr/bin/env python3
"""Main entry point — runs GA training for geometric and neural-net controllers."""

import argparse
import json
import multiprocessing
import os
import time

from controllers import GeometricController, NeuralNetController
from ga import GeneticAlgorithm
from sim import run_episode


def main():
    parser = argparse.ArgumentParser(description="Train kart controllers with GA")
    parser.add_argument("--generations", type=int, default=50)
    parser.add_argument("--pop-size", type=int, default=100)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--output-dir", type=str, default="results")
    parser.add_argument("--geometric-only", action="store_true")
    parser.add_argument("--neural-only", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    gas = {}
    if not args.neural_only:
        gas["geometric"] = GeneticAlgorithm(GeometricController,
                                            pop_size=args.pop_size)
    if not args.geometric_only:
        gas["neural"] = GeneticAlgorithm(NeuralNetController,
                                         pop_size=args.pop_size)

    print(f"Training {list(gas.keys())} | "
          f"generations={args.generations}  pop={args.pop_size}  "
          f"workers={args.workers}\n")

    for gen in range(args.generations):
        t0 = time.time()
        for name, ga in gas.items():
            ga.evaluate(workers=args.workers)
            s = ga.stats()
            dt = time.time() - t0
            print(f"[{name:>10}] gen {gen:3d} | "
                  f"best={s['best']:8.1f}  avg={s['avg']:8.1f} | "
                  f"all-time={s['all_time_best']:8.1f} | "
                  f"σ={s['sigma']:.4f} | {dt:.1f}s")
            ga.evolve()
        print()

    # Save best controllers
    for name, ga in gas.items():
        ctrl = ga.controller_class(ga.best_genes)
        result = run_episode(ctrl)

        payload = {
            "controller_type": name,
            "genes": ga.best_genes.tolist(),
            "fitness": ga.best_fitness,
            "result": result,
            "generations": args.generations,
            "pop_size": args.pop_size,
        }

        path = os.path.join(args.output_dir, f"best_{name}.json")
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)

        print(f"Saved {name} → {path}")
        print(f"  fitness={ga.best_fitness:.1f}  "
              f"dist={result['distance']:.1f}m  laps={result['laps']}  "
              f"avg_cte={result['avg_cte']:.2f}m  avg_speed={result['avg_speed']:.1f}m/s")
        if name == "geometric":
            g = ga.best_genes
            print(f"  genes: gain={g[0]:.3f} max_steer={g[1]:.3f} "
                  f"max_speed={g[2]:.3f} min_speed={g[3]:.3f} "
                  f"lookahead={g[4]:.3f} curve_factor={g[5]:.3f}")
        print()


if __name__ == "__main__":
    multiprocessing.set_start_method("fork")
    main()
