# pool_physics

A pure Dart billiards physics engine whose numerical outputs are constrained by
Dr. Dave Alciatore's experimental results.  The simulation models cue-tip
impact, ball–cloth interaction, ball–ball collision, and three-dimensional
cushion rebound.  It produces annotated trajectories suitable for shot-diagram
renderers, teaching tools, or analysis pipelines.

## Philosophy

The engine is not a first-principles particle simulator.  It is a set of
calibrated analytic models, each anchored to a published measurement.  Every
tunable parameter has a cited source; every model boundary is tested.  See
[`doc/philosophy.md`](doc/philosophy.md) for the full rationale.

## Quick start

```dart
import 'package:pool_physics/pool_physics.dart';

void main() {
  final result = simulate(
    balls: [
      PhysicsBall.at('cue', const Vec2(0.25, 0.50)),
      PhysicsBall.at('nine', const Vec2(0.75, 0.50)),
    ],
    cueBallId: 'cue',
    shot: const ShotParams(
      aimDir: Vec2(1, 0),
      speedMs: 2.0,
      hitX: 0.0,
      hitY: 0.35, // slight follow
    ),
    tableConfig: PhysicsTableConfig.nineFootDiamond,
  );

  print('cue ends at ${result.endPositions['cue']}');
  print('nine ends at ${result.endPositions['nine']}');
  print('cushion contacts: ${result.cushionContacts}');
}
```

See [`example/straight_shot.dart`](example/straight_shot.dart) for a
self-contained runnable example with annotated output.

## Coordinate system

All positions are in **normalized table coordinates**:

- `(0, 0)` is the top-left corner of the table bed (including rails).
- `(1, 1)` is the bottom-right corner.
- The playable surface begins at `(railW, railW)` and ends at
  `(1 − railW, 1 − railH)`, where `railW = 7/114 ≈ 0.0614` and
  `railH = 7/64 ≈ 0.1094` for a nine-foot Diamond.

`ShotParams.aimDir` is a unit vector in the same normalized space; the
physical aspect ratio is accounted for internally.

`PhysicsTableConfig.playW` and `playH` hold the actual play-surface dimensions
in metres and are used to convert normalized distances to physical quantities.

## Core types

| Type | Purpose |
|---|---|
| `PhysicsBall` | A ball with an id and normalized (x, y) position. |
| `ShotParams` | Cue speed, aim direction, tip offset, elevation, and cue type. |
| `PhysicsTableConfig` | Table geometry, cloth friction coefficients, and elevation rules. |
| `CushionProfile` | Rail nose geometry and restitution coefficients. |
| `PhysicsResult` | All trajectories, contact details, and final positions returned by `simulate`. |
| `PhysicsPath` | One ball's position samples for a single phase of motion. |
| `PhysicsPathStyle` | Whether a path segment is the cue before contact, after contact, or an object-ball roll. |
| `Vec2` / `Vec3` | Immutable 2-D and 3-D vectors, independent of Flutter. |

## `simulate`

```dart
PhysicsResult simulate({
  required List<PhysicsBall> balls,
  required String cueBallId,
  required ShotParams shot,
  required PhysicsTableConfig tableConfig,
  int shotIndex = 0,
  CushionProfile cushion = CushionProfile.standardPool,
  bool forPreview = false,
  bool forSolver = false,
  double? maxTimeSeconds,
  double ballBallFrictionScale = 1.0,
  double ballBallRestitution = 0.93,
})
```

`simulate` is the single entry point.  It integrates the full shot from
cue-tip contact through all collisions and rail rebounds until every ball
stops or pockets.

`forSolver: true` keeps physics and contact/path results but skips animation
frames and research-only trajectory diagnostics, reducing allocation during
large candidate searches. `ballBallRestitution` defaults to the calibrated
baseline (`0.93`) and can be adjusted for controlled comparisons. The value
must be finite and between 0 and 1.

### `ShotParams`

| Field | Type | Description |
|---|---|---|
| `aimDir` | `Vec2` | Unit vector in normalized-coordinate space pointing toward the target. |
| `speedMs` | `double` | Cue-ball speed in m/s immediately after tip separation. |
| `hitX` | `double` | Horizontal tip offset in units of ball radius R, range −0.5 to +0.5. Positive is right. |
| `hitY` | `double` | Vertical tip offset in units of R, range −0.5 to +0.5. Positive is top-spin. |
| `elevationDeg` | `double?` | Override the computed cue elevation (degrees above the cloth). Null means auto-compute. |
| `cueType` | `CueType` | `CueType.lowDeflection` (default) or `CueType.standard`. Affects squirt magnitude. |

### `PhysicsResult`

| Field | Description |
|---|---|
| `paths` | All position-sample sequences, one per ball per motion phase. |
| `endPositions` | Final resting position for each ball id, or `null` if pocketed. |
| `cushionContacts` | Total number of ball–rail impact events. |
| `cushionContactDetails` | Per-impact detail: rail, incident/rebound angles, spin transfer. |
| `ballBallContactDetails` | Per-collision detail: throw angle, impulse components, pre/post velocities. |
| `peakSpinRpm` | Highest topspin/draw rpm reached by each ball during the shot. |
| `peakSpinZ` | Highest vertical-axis (sidespin) angular velocity (rad/s) for each ball. |
| `ghostBallRel` | Normalized position of the ghost-ball contact for the first cue–object collision. |
| `finalLinearVelocities` | Linear velocity (m/s in physical coordinates) for each ball at the final timestep. |

## Physics models

Full citations are in [`doc/tp_index.md`](doc/tp_index.md).  A brief summary:

**Cue–tip impact** — The tip offset determines ball speed, topspin, and
sidespin via the transfer-function equations in TP A.12, A.22, A.30, and B.7.
Squirt is applied before the ball leaves the tip (TP A.31 / B.7).

**Ball–cloth interaction** — Sliding decelerates according to the kinetic
friction coefficient (`clothSlidingFriction`); rolling resistance applies once
the ball reaches natural roll.  Spin decay is calibrated to measured values
from Dr. Dave's TP series.

**Ball–ball collision** — A single-impulse model with restitution `e = 0.93`
and the friction–slip curve from Dr. Dave and Marlow (TP A.14):
`μ = 0.009951 + 0.108·exp(−1.088·v)`.  Touching chains use a finite-contact
Hertz integrator (TP B.29).

**Cushion rebound** — The Mathavan et al. 3-D model.  The cushion nose lifts
the ball slightly at impact, transferring a fraction of the incoming velocity
into vertical-axis spin.  Restitution is speed-dependent.  Two built-in
profiles are provided: `CushionProfile.standardPool` and
`CushionProfile.diamondRedLabel`.

**Massé and swerve** — Elevated shots track the ball's height above the cloth
through the parabolic arc set by the cue elevation angle, then apply the cloth
deceleration once the ball lands.  Swerve builds from the vertical-axis spin
component created at impact.

## Limitations

- The cloth models assume a level, flat surface.
- Jumps over obstacles are not modelled; elevation only affects the cue-ball
  path after it lands.
- Jawed balls and ball-in-hand rules are not included.
- The model is calibrated for standard pool (6 × 3 ft to 9 × 4.5 ft tables
  with 2¼-inch balls).  Snooker geometry has not been validated.

## Attribution

The physics constants and validation bounds in this package are derived from
work by **Dr. Dave Alciatore** (billiards.colostate.edu, TP series), whose
experimental measurements constrain every major model parameter.  See
[`doc/tp_index.md`](doc/tp_index.md) for a complete citation index.

The three-dimensional cushion contact model follows **Mathavan, Jackson, and
Parkin** (2010), "A theoretical analysis of billiard ball dynamics under
cushion impacts," _Proceedings of the Institution of Mechanical Engineers,
Part C_.

The Hertz compliance constant for ball chains is taken from **Marlow** (1995),
_The Physics of Pocket Billiards_.

## License

MIT — see [`LICENSE`](LICENSE).
