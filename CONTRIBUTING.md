# Contributing Guidelines

This repository uses a **staging → main** workflow to ensure all code is safe, tested, and competition-ready.

## Branches

### `staging`
- Used for **simulation-tested code**
- All new features, fixes, and changes start here
- Code must:
  - Build successfully
  - Run correctly in simulation
- Code in `staging` is **not guaranteed** to be safe for the real robot

### `main`
- **Production / competition branch**
- Must always be safe to deploy to the robot
- **Only accepts pull requests from `staging`**
- Code must:
  - Be tested on **real robot hardware**
  - Be reviewed and approved before merging

> Direct commits or PRs from feature branches to `main` are **not allowed**.

## Pull Request Rules

- All PRs require **at least 2 reviewers**
- Reviewers should understand the code being changed
- Do not approve your own PR
- Address all review comments before merging

## Workflow Summary

1. Create a feature branch from `staging`
2. Develop and test your code in simulation
3. Open a PR **into `staging`**
4. After testing on real hardware:
   - Open a PR from `staging` → `main`
5. Get **2 approvals** (one reviewer must be @logandhillon or @anvaymathur)
6. Merge only when all requirements are met

## Testing Requirements

Before merging into `main`, confirm:
- [ ] Code was tested on the real robot
- [ ] No simulation-only hacks remain
- [ ] Robot behavior is verified and safe

## General Rules

- Keep `main` clean and deployable at all times
- Small, focused PRs are preferred
- If you are unsure, ask before merging
