# Physics Philosophy

## Calibrated models, not particle simulation

This engine does not attempt to derive billiard ball trajectories from first
principles.  A from-scratch rigid-body simulation would need sub-millisecond
time steps, accurate cloth-fiber contact geometry, and validated material
constants — none of which are practically measurable for a consumer table.

Instead, every sub-model is an analytic equation whose free parameters are set
from published experimental measurements.  The design rule is:

> **A parameter may be tuned to match an experiment; it may not be tuned to
> make a specific shot look correct.**

If a model disagrees with a measurement it is supposed to cover, the model is
wrong, not the measurement.  This constraint is enforced through the test suite:
every calibration value appears in at least one test that will fail if the
value drifts.

## Dr. Dave Alciatore's TP series as the validation corpus

**Dr. Dave Alciatore** is a professor of mechanical engineering at Colorado
State University and the author of the "Technical Proofs" (TP series) at
billiards.colostate.edu.  The series applies engineering mechanics to pool and
covers cue-tip physics, cloth friction, ball–ball collisions, cushion rebound,
massé, and more — each proof matched against measured data.

The TP series is the engine's primary validation corpus.  For each major model:

1. The paper's prediction is implemented as a formula.
2. A test feeds the same inputs to `simulate` and compares the output to the
   formula's output — not to a hard-coded number chosen to match the sim.
3. The tolerance bounds come from the paper's own stated accuracy, not from
   trial-and-error tuning.

The goal is that someone familiar with Dr. Dave's work can read a test, look up
the cited TP, and confirm independently that the expected value is correct.

Attribution is by citation only.  No equations or figures from the TP series
are reproduced verbatim in this codebase.  The specific TPs consulted are
indexed in [`tp_index.md`](tp_index.md).

## One model per phenomenon

Billiard physics has many interacting effects.  The engine keeps them
separated:

| Phenomenon | Model | Where |
|---|---|---|
| Cue–tip energy transfer | Eccentric impact equations | `cue_impact.dart` |
| Squirt | Tip-offset deflection formula (TP A.31 / B.7) | `cue_impact.dart` |
| Ball sliding on cloth | Coulomb kinetic friction | `sim.dart` |
| Ball rolling on cloth | Rolling resistance + vertical-axis decay | `sim.dart` |
| Massé / swerve | 3-D parabolic arc + cloth landing | `sim.dart` |
| Ball–ball collision | Single impulse + Alciatore friction curve | `sim.dart` |
| Touching-ball chain | Damped Hertz finite-contact integrator | `hertz_contact.dart` |
| Cushion rebound | Mathavan et al. 3-D model | `sim.dart` |
| Table geometry | Rail dimensions from Diamond specs | `table_constants.dart` |

Each model is self-contained.  If the community publishes a better cushion
model, it can be swapped in without touching the cloth or impact code.

## What the engine is not

**Not an AI shot recommender.**  `simulate` is a forward model: given a shot,
it produces a trajectory.  Picking a good shot requires inverting that model,
which this package does not do.

**Not a rendering library.**  The `PhysicsPath` sequences are position samples
at fixed time intervals.  How to draw them — smoothed Bézier curves, simple
lines, animated sprites — is entirely up to the caller.

**Not a rules engine.**  Pocketing detection terminates a ball's trajectory.
Whether that pocket counts, who shoots next, and what the score is are not
modelled.

**Not validated for snooker.**  The cloth friction range, ball restitution, and
cushion geometry are calibrated for pool (2¼-inch balls, standard felt,
pool-table cushion profiles).  Snooker uses smaller balls, faster cloth, and
different pocket geometry; the same constants will not apply.

## Tolerance philosophy

Most tests assert `closeTo(expected, tolerance)`.  The tolerances are set to
roughly **2×** the model's own stated accuracy — wide enough to survive
minor floating-point drift across platforms, tight enough to catch a wrong
formula or an off-by-ten constant.

Tolerances are **not** set from "what the sim currently outputs."  They are set
from the paper's stated accuracy, and the sim is expected to meet them.  A test
that fails because the sim is wrong should be fixed by improving the model, not
by widening the tolerance.

## Extending the engine

If you add a new physical effect:

1. Find a peer-reviewed or rigorously measured source for the parameter values.
2. Write the test first, citing the source in the test name (e.g.
   `'TP A.14 ...'`).
3. Implement the model until the test passes.
4. Run `dart analyze --fatal-infos` and `dart format`.

If the only justification for a constant is "it looked about right," it does
not belong in this package.
