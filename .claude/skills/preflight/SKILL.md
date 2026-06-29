---
name: preflight
description: Inspect SC2026 repository state, relevant documentation, environment boundaries, and validation requirements before starting a task.
disable-model-invocation: true
allowed-tools: Read Grep Glob Bash(git status*) Bash(git branch --show-current) Bash(git log*) Bash(git diff*) Bash(git ls-files*)
---

# SC2026 Task Preflight

Perform a read-only assessment before implementation.

## Procedure
1. Run `git status --short --branch`.
2. Confirm the current branch is not `main`.
3. Identify existing modified and untracked files. Do not assume they belong to the current task.
4. Read `CLAUDE.md` and the relevant files under `Env/`, `Notice/`, or `D-Racer-Kit/docs/`.
5. Inspect the smallest relevant source area.
6. Classify each validation step as:
   - macOS local
   - Linux Server PC
   - D3-G stationary
   - D3-G wheels-off-ground
   - D3-G track
7. Identify commands that require explicit approval.
8. Do not edit files, install packages, commit, or push.

## Output
Report:
- Current branch and working-tree state
- Task interpretation and success criteria
- Relevant files and interfaces
- Required execution environment
- Proposed validation sequence
- Safety or regression risks
- The single recommended next action