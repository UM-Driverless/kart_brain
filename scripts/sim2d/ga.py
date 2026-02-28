"""Genetic algorithm: evaluate, select, crossover, mutate."""

import numpy as np
from multiprocessing import Pool

from sim import run_episode

# Module-level fitness_mode so worker processes can see it (set by GA)
_fitness_mode = "v1"


def _evaluate_one(args):
    """Top-level worker function (must be picklable for multiprocessing)."""
    controller_class, genes = args
    ctrl = controller_class(genes)
    return run_episode(ctrl, fitness_mode=_fitness_mode)["fitness"]


class GeneticAlgorithm:
    """Standard GA with tournament selection, uniform crossover, Gaussian mutation."""

    def __init__(self, controller_class, pop_size=100, elite_size=5,
                 tournament_k=5, mutation_sigma=0.1, mutation_decay=0.995,
                 fitness_mode="v1"):
        self.controller_class = controller_class
        self.pop_size = pop_size
        self.elite_size = elite_size
        self.tournament_k = tournament_k
        self.mutation_sigma = mutation_sigma
        self.mutation_decay = mutation_decay
        self.num_genes = controller_class.NUM_GENES
        self.fitness_mode = fitness_mode

        self.population = self._init_population()
        self.fitnesses = np.zeros(pop_size)
        self.generation = 0
        self.best_fitness = -np.inf
        self.best_genes = None

    # ── initialisation ────────────────────────────────────────────────

    def _init_population(self):
        if hasattr(self.controller_class, "RANGES"):
            # Geometric: sample uniformly inside gene ranges, keep defaults
            ranges = self.controller_class.RANGES
            pop = np.random.uniform(
                ranges[:, 0], ranges[:, 1],
                size=(self.pop_size, self.num_genes),
            )
            pop[0] = self.controller_class.DEFAULTS  # ensure defaults in pool
            return pop
        # Neural net: Xavier-like
        scale = 1.0 / np.sqrt(self.controller_class.INPUT_SIZE)
        return np.random.randn(self.pop_size, self.num_genes) * scale

    # ── evaluation ────────────────────────────────────────────────────

    def evaluate(self, workers=1):
        global _fitness_mode
        _fitness_mode = self.fitness_mode

        args = [(self.controller_class, self.population[i])
                for i in range(self.pop_size)]

        if workers > 1:
            with Pool(workers) as pool:
                self.fitnesses = np.array(pool.map(_evaluate_one, args))
        else:
            self.fitnesses = np.array([_evaluate_one(a) for a in args])

        best_idx = int(np.argmax(self.fitnesses))
        if self.fitnesses[best_idx] > self.best_fitness:
            self.best_fitness = float(self.fitnesses[best_idx])
            self.best_genes = self.population[best_idx].copy()

    # ── selection / variation ─────────────────────────────────────────

    def _tournament(self):
        idxs = np.random.choice(self.pop_size, self.tournament_k, replace=False)
        return self.population[idxs[np.argmax(self.fitnesses[idxs])]]

    def _crossover(self, p1, p2):
        mask = np.random.random(self.num_genes) < 0.5
        return np.where(mask, p1, p2)

    def _mutate(self, ind):
        child = ind + np.random.randn(self.num_genes) * self.mutation_sigma
        if hasattr(self.controller_class, "RANGES"):
            lo = self.controller_class.RANGES[:, 0]
            hi = self.controller_class.RANGES[:, 1]
            child = np.clip(child, lo, hi)
        return child

    # ── evolution step ────────────────────────────────────────────────

    def evolve(self):
        order = np.argsort(-self.fitnesses)
        new_pop = np.empty_like(self.population)

        # Elitism
        new_pop[:self.elite_size] = self.population[order[:self.elite_size]]

        # Offspring
        for i in range(self.elite_size, self.pop_size):
            new_pop[i] = self._mutate(self._crossover(
                self._tournament(), self._tournament()))

        self.population = new_pop
        self.mutation_sigma *= self.mutation_decay
        self.generation += 1

    # ── reporting ─────────────────────────────────────────────────────

    def stats(self):
        return {
            "generation": self.generation,
            "best": float(np.max(self.fitnesses)),
            "avg": float(np.mean(self.fitnesses)),
            "worst": float(np.min(self.fitnesses)),
            "all_time_best": self.best_fitness,
            "sigma": self.mutation_sigma,
        }
