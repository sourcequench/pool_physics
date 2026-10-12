import 'vec2.dart';
import 'vec3.dart';

// ── Path styles ───────────────────────────────────────────────────────────────

/// Identifies a path segment's role in the shot sequence.
///
/// Ordinal values are stable and must not be reordered.
enum PhysicsPathStyle {
  /// Cue-ball path before it contacts an object ball.
  cueBefore,

  /// Cue-ball path after it contacts an object ball.
  cueAfter,

  /// Any object ball's path.
  objectBall,

  /// Ghost-ball overlay path (aim preview only).
  ghostBall,
}

// ── Ball ──────────────────────────────────────────────────────────────────────

/// A ball's initial state, expressed in relative table coordinates.
///
/// `x` and `y` are in `[0, 1]` space where `(0, 0)` is the table's top-left
/// outer corner. Use [PhysicsTableConfig] to convert between relative and
/// metric positions.
class PhysicsBall {
  final String id;
  final double x;
  final double y;
  final bool pocketed;

  const PhysicsBall({
    required this.id,
    required this.x,
    required this.y,
    this.pocketed = false,
  });

  /// Convenience constructor from a [Vec2] position.
  factory PhysicsBall.at(String id, Vec2 position, {bool pocketed = false}) =>
      PhysicsBall(id: id, x: position.x, y: position.y, pocketed: pocketed);

  Vec2 get position => Vec2(x, y);
}

// ── Table config ──────────────────────────────────────────────────────────────

/// Play-surface dimensions and cloth friction coefficients for a specific table.
///
/// `playW` and `playH` are the playable surface dimensions in metres — the
/// area inside the cushion faces, not the outer frame. All pre-defined
/// constants ([nineFootDiamond] etc.) model Diamond Pro-Am tables.
///
/// Cloth friction defaults match Dr. Dave's published mid-range values:
/// μ_s = 0.20 (sliding), μ_r = 0.010 (rolling). Adjust these to model
/// specific cloth conditions.
class PhysicsTableConfig {
  /// Play surface width, metres (long axis).
  final double playW;

  /// Play surface height, metres (short axis).
  final double playH;

  final double minimumCueElevationDeg;

  /// Coulomb sliding friction coefficient (cloth). [Dr. Dave TP B.2]
  final double clothSlidingFriction;

  /// Rolling resistance coefficient (cloth). [Dr. Dave TP B.12]
  final double clothRollingResistance;

  const PhysicsTableConfig({
    required this.playW,
    required this.playH,
    this.minimumCueElevationDeg = 3.0,
    this.clothSlidingFriction = 0.20,
    this.clothRollingResistance = 0.010,
  });

  static const nineFootDiamond = PhysicsTableConfig(playW: 2.540, playH: 1.270);
  static const eightFootDiamond = PhysicsTableConfig(
    playW: 2.235,
    playH: 1.118,
  );
  static const sevenFootDiamond = PhysicsTableConfig(
    playW: 1.930,
    playH: 0.965,
  );
}

// ── Path ─────────────────────────────────────────────────────────────────────

class PhysicsPath {
  final String ballId;
  final List<Vec2> points;
  final PhysicsPathStyle style;
  final int shotIndex;

  const PhysicsPath({
    required this.ballId,
    required this.points,
    required this.style,
    this.shotIndex = 0,
  });
}

// ── Cushion contact ───────────────────────────────────────────────────────────

class CushionContact {
  final double timeSec;
  final String ballId;
  final String rail;
  final Vec2 positionRel;
  final double incidentAngleDeg;
  final double reboundAngleDeg;
  final double incidentSpeedMs;
  final double reboundSpeedMs;
  final double spinRpmBefore;
  final double spinRpmAfter;
  final double spinZBefore;
  final double spinZAfter;

  const CushionContact({
    required this.timeSec,
    required this.ballId,
    required this.rail,
    required this.positionRel,
    required this.incidentAngleDeg,
    required this.reboundAngleDeg,
    required this.incidentSpeedMs,
    required this.reboundSpeedMs,
    required this.spinRpmBefore,
    required this.spinRpmAfter,
    required this.spinZBefore,
    required this.spinZAfter,
  });
}

// ── Pocket-facing contact ──────────────────────────────────────────────────────────────────

class PocketContact {
  final double timeSec;
  final String ballId;
  final String pocket;
  final String surface;
  final Vec2 positionRel;
  final double incidentSpeedMs;
  final double reboundSpeedMs;

  const PocketContact({
    required this.timeSec,
    required this.ballId,
    required this.pocket,
    required this.surface,
    required this.positionRel,
    required this.incidentSpeedMs,
    required this.reboundSpeedMs,
  });
}

class BallBallContact {
  final double timeSec;
  final String cueBallId;
  final String objectBallId;
  final Vec2 impactNormal;
  final Vec3 impactNormal3;
  final double incomingNormalSpeedMs;
  final Vec3 relativeTangentialSlip;
  final double relativeTangentialSlipSpeedMs;
  final double frictionCoefficient;
  final double normalImpulsePerMass;
  final double tangentialImpulsePerMass;
  final Vec3 tangentialImpulseOnCueBall;
  final double throwAngleRad;
  // Pre/post states and the resolved 3-D tangential impulse make impact
  // diagnostics auditable against TP A.14's sliding-velocity/impulse model.
  final Vec3 cuePreImpactVelocity;
  final Vec3 cuePreImpactAngularVelocity;
  final Vec3 cuePostImpactVelocity;
  final Vec3 cuePostImpactAngularVelocity;
  final Vec3 objectPostImpactAngularVelocity;

  const BallBallContact({
    required this.timeSec,
    required this.cueBallId,
    required this.objectBallId,
    required this.impactNormal,
    required this.impactNormal3,
    required this.incomingNormalSpeedMs,
    required this.relativeTangentialSlip,
    required this.relativeTangentialSlipSpeedMs,
    required this.frictionCoefficient,
    required this.normalImpulsePerMass,
    required this.tangentialImpulsePerMass,
    required this.tangentialImpulseOnCueBall,
    required this.throwAngleRad,
    required this.cuePreImpactVelocity,
    required this.cuePreImpactAngularVelocity,
    required this.cuePostImpactVelocity,
    required this.cuePostImpactAngularVelocity,
    required this.objectPostImpactAngularVelocity,
  });
}

// ── Result ────────────────────────────────────────────────────────────────────

class PhysicsResult {
  final List<PhysicsPath> paths;
  final Map<String, Vec2?> endPositions;
  final Map<String, Vec2> pocketPositions;
  final Map<String, double> endHeights;
  final Map<String, double> peakHeights;
  final Map<String, int> tableBounceCounts;
  // Horizontal distance from the shot start to each table landing. Useful for
  // comparing elevated-shot bounce sequences against measured/reference data.
  final Map<String, List<double>> tableBounceDistancesMeters;
  final Map<String, List<double>> tableBounceSpeedsMs;
  final Map<String, List<double>> tableBounceSpinRps;
  // Absolute Rωz / forward-speed ratio at the first sliding-to-rolling
  // transition; useful for validating drag-enhanced sidespin (Dr. Dave TP B.17).
  final Map<String, double> sideSpinRatioAtRoll;
  final Map<String, double> peakSpinZ;
  final Map<String, double> peakSpinRpm;

  /// Final 3-D translational velocity of each ball, for physics diagnostics.
  final Map<String, Vec3> finalLinearVelocities;
  final int cushionContacts;
  final List<CushionContact> cushionContactDetails;
  final List<PocketContact> pocketContactDetails;
  final List<BallBallContact> ballBallContactDetails;
  final Set<String> cornerShelfBalls;
  final Vec2? ghostBallRel;
  final double cueElevationDeg;
  // Ball positions at each recorded timestep (every 8 ms). Each frame maps
  // ballId → relative position for all non-pocketed balls at that instant.
  final List<Map<String, Vec2>> frames;

  const PhysicsResult({
    required this.paths,
    required this.endPositions,
    this.pocketPositions = const {},
    this.endHeights = const {},
    this.peakHeights = const {},
    this.tableBounceCounts = const {},
    this.tableBounceDistancesMeters = const {},
    this.tableBounceSpeedsMs = const {},
    this.tableBounceSpinRps = const {},
    this.sideSpinRatioAtRoll = const {},
    this.peakSpinZ = const {},
    this.peakSpinRpm = const {},
    this.finalLinearVelocities = const {},
    this.cushionContacts = 0,
    this.cushionContactDetails = const [],
    this.pocketContactDetails = const [],
    this.ballBallContactDetails = const [],
    this.cornerShelfBalls = const {},
    this.ghostBallRel,
    this.cueElevationDeg = 3.0,
    this.frames = const [],
  });
}
