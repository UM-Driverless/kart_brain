# Kart SW Agent Documentation

This directory contains persistent AI-focused documentation for the kart_sw workspace. It follows a two-layer system: **memory** (what agents read) and **enforcement** (what prevents bad work).

## Why This Exists

LLMs have no persistent memory between sessions. Every conversation starts fresh. This directory captures hard-won knowledge about the system so it doesn't have to be rediscovered each time.

## File Index

### Layer 1: Memory (What Agents Read)

| File | Purpose |
|---|---|
| `architecture.md` | Package structure, node graph, message types, topic map |
| `simulation.md` | Gazebo setup, known issues, rendering quirks, how to test |
| `vm_environment.md` | UTM VM specifics: SSH, sudo, installed packages, limitations |
| `error_log.md` | Running log of mistakes and prevention mechanisms |
| `postmortems/` | Detailed failure analysis for significant errors |

### Layer 2: Enforcement

| File | Purpose |
|---|---|
| `../scripts/` | Workspace-level utility scripts |

## Workflow

1. **Before starting:** Read `AGENTS.md` (root) → relevant `.agents/` files
2. **During work:** Follow architecture conventions from `architecture.md`
3. **Before commit:** Build, verify, `git status` + `git diff`
4. **If something breaks:** Document in `error_log.md`, add prevention

## Key Principles

- **Document what was painful** — If you spent time debugging something, write it down
- **Cone class IDs matter** — The whole pipeline depends on consistent string IDs (`blue_cone`, `yellow_cone`, etc.)
- **Gazebo Fortress ≠ modern Gazebo** — Uses `ign` CLI, `ignition.msgs.*` types, and lacks some SDF features (no `<cone>` geometry)
- **No GPU** — Everything renders via LLVMpipe. Keep resolutions low, disable shadows
- **Odom is relative** — Gazebo odometry starts at (0,0), not at the world pose. Always account for the initial spawn position
