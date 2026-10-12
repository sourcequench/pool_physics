# Contributing

## Before you start

Read [`doc/philosophy.md`](doc/philosophy.md).  The engine is a set of
calibrated analytic models, not a general-purpose simulator.  Every numerical
constant needs a cited source; parameters are not tuned to make shots "look
right."

If you are proposing a new physical effect, locate a peer-reviewed or
rigorously measured source before writing any code.

## Development setup

```bash
git clone https://github.com/ryanshea10/pool_physics
cd pool_physics
dart pub get
```

Requires the Dart SDK ≥ 3.3.0.  No Flutter dependency.

## Running tests

```bash
dart test
```

All tests must pass before a pull request is accepted.  The test suite covers
the numerical predictions from the TP series; a failing test means a model
regression, not a tolerance that needs widening.

## Code quality

```bash
dart analyze --fatal-infos
dart format --set-exit-if-changed .
```

Both commands are enforced in CI.  Submissions that introduce analyzer
warnings or formatting drift will not be merged.

## Adding a new model or constant

1. **Find a source.** Peer-reviewed papers or Dr. Dave's TP series are
   acceptable.  "Felt about right in testing" is not.

2. **Write the test first.** Name it after the source
   (e.g. `'TP A.14 ...'`).  Set the tolerance from the paper's stated
   accuracy, not from the sim's current output.

3. **Implement the model** until the new test and all existing tests pass.

4. **Update [`doc/tp_index.md`](doc/tp_index.md)** with the new citation.

5. **Run `dart analyze --fatal-infos` and `dart format`.**

## Pull request checklist

- [ ] All tests pass (`dart test`)
- [ ] No analyzer issues (`dart analyze --fatal-infos`)
- [ ] Code is formatted (`dart format --set-exit-if-changed .`)
- [ ] New constants are cited in `doc/tp_index.md`
- [ ] Tolerances come from the cited source, not from trial and error

## Reporting bugs

Open a GitHub issue with:
- The `simulate` call that reproduces the problem (minimal example preferred).
- The expected physical behaviour and its source.
- The observed output.

Feature requests without a cited physical source are unlikely to be accepted.
