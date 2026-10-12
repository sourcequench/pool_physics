/// A validated billiards physics simulation.
///
/// Import this library for the complete public API. The primary entry point is
/// [simulate]; see [ShotParams] and [PhysicsTableConfig] for input types and
/// [PhysicsResult] for output.
///
/// All positions use relative table coordinates: `(0, 0)` is the top-left
/// outer corner, `(1, 1)` is the bottom-right outer corner. The play surface
/// occupies roughly `(0.061, 0.109)` to `(0.939, 0.891)` on a nine-foot
/// Diamond table.
library;

export 'src/cue_impact.dart';
export 'src/hertz_contact.dart';
export 'src/sim.dart';
export 'src/table_constants.dart';
export 'src/types.dart';
export 'src/validation.dart';
export 'src/vec2.dart';
export 'src/vec3.dart';
