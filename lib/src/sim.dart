// Billiards physics simulation.
//
// Works in metres on the play surface; uses relative (0–1) coords for I/O.
// Pure Dart — no Flutter dependencies.
//
// ── Public API overview ───────────────────────────────────────────────────────
// simulate()           — run a shot from cue impact to final rest
// ShotParams           — cue-tip contact point, speed, spin, and cue type
// CushionProfile       — rail geometry and friction coefficients
// CueType              — low-deflection vs. standard cue models
// computeElevationDeg  — minimum cue elevation for a given shot geometry
// ballBallFrictionForSlip — Alciatore ball-ball friction curve (TP A.14)
// cueSquirtAngleRad    — squirt angle from TP A.31 effective-endmass model
// cueEffectiveLateralOffsetR — spin-producing offset after squirt (TP B.7)
// spinZDecayRateRadPerSec2   — vertical-axis spin drag constant
// maxPhysicalSpinRpm         — upper bound on achievable RPM
import 'dart:math' as math;

import 'cue_impact.dart';
import 'table_constants.dart';
import 'hertz_contact.dart';
import 'types.dart';
import 'vec2.dart';
import 'vec3.dart';

// ── Physical constants (Dr. Dave / Alciatore / Mathavan et al.) ─────────────

// ignore: constant_identifier_names — single-letter physics notation for ball radius
const double _R = 0.028575; // ball radius (m)
const double _ballInertia = 2.0 / 5.0; // I / (m R²) for a solid sphere
const double _eBall = 0.93; // ball–ball restitution
const double _eTable = 0.60; // Dr. Dave's typical ball-table range: 0.5–0.7
const double _g = 9.81; // gravity (m/s²)

const double spinZDecayRateRadPerSec2 = 10.0;
const double maxPhysicalSpinRpm = 2500.0;
const double _radiansPerSecondToRpm = 60.0 / (2.0 * math.pi);

// TP 2.1 gives μ=0.577 as the no-slip threshold at 0.5R. This models a
// well-chalked tip at the generally recommended safe offset; real chalk/tip
// friction varies, so this is a deterministic baseline, not a universal limit.
const double _assumedCueTipFriction = 0.5773502691896257;
final double _miscueLimit = maximumNoSlipCueTipOffsetR(_assumedCueTipFriction);

const double _cueBallMassKg = 0.170;
// Effective cue end masses chosen to reproduce Dr. Dave's representative
// maximum-offset squirt angles (about 1.8° LD, 2.5° standard); both are within
// the 5–15 g range discussed in TP A.31.
const double _endMassLowDeflectionKg = 0.0054;
const double _endMassStandardKg = 0.0078;

// For a solid sphere, TP A.12 gives SRF = (Rω)/V = (5/2)(b/R) for
// longitudinal (top/bottom) spin, where b/R is the normalized tip offset.
const double _spinZFactor = cueBallSpinRatioPerOffset;
const double _spinYFactor = cueBallSpinRatioPerOffset;

const double _dt = 0.001;
const double _recordEvery = 0.008;
// Slow natural-roll shots on fast cloth can take longer than 12 seconds to
// settle, especially on large unobstructed validation tables (e.g. Dr. Dave
// TP B.12's 30 ft travel example). Normal shots still exit as soon as all
// balls stop; this is only the safety horizon for unusually long paths.
const double _maxTime = 30.0;

const double _relX0 = railWidth;
const double _relX1 = 1.0 - railWidth;
const double _relY0 = railHeight;
const double _relY1 = 1.0 - railHeight;

// ── Table config ─────────────────────────────────────────────────────────────

class _Cfg {
  final double W;
  final double H;
  final double minimumCueElevationDeg;
  final double clothSlidingFriction;
  final double clothRollingResistance;

  const _Cfg(
    this.W,
    this.H,
    this.minimumCueElevationDeg,
    this.clothSlidingFriction,
    this.clothRollingResistance,
  );

  double get cornerJawR => cornerMouthInches / math.sqrt2 * inchesToMeters;
  double get cornerHoleR => cornerHoleRadiusInches * inchesToMeters;
  double get cornerShelf => cornerShelfInches * inchesToMeters;
  double get cornerFacingAngle => cornerFacingAngleDegrees * math.pi / 180.0;
  double get cornerHoleOffset {
    final offsetInches =
        (cornerShelfInches + cornerHoleRadiusInches - cornerMouthInches / 2) /
            math.sqrt2;
    return offsetInches * inchesToMeters;
  }

  double get cornerFacingLength {
    final offset = cornerHoleOffset;
    final leg = cornerJawR;
    final cosA = math.cos(cornerFacingAngle);
    final sinA = math.sin(cornerFacingAngle);
    final projection = (leg + offset) * cosA + offset * sinA;
    final distanceSquared = (leg + offset) * (leg + offset) + offset * offset;
    return projection -
        math.sqrt(
          projection * projection -
              (distanceSquared - cornerHoleR * cornerHoleR),
        );
  }

  // The rail nose is suppressed only where the rounded jaw/facing geometry
  // takes over. This includes one ball radius beyond the mouth tip.
  double get cornerPocketDetectR => cornerJawR + _R * 1.2;
  double get sidePocketHalfW => 0.076 * W / 2.54;
  double get sidePocketDepth => 0.030 * W / 2.54;

  static _Cfg from(PhysicsTableConfig config) => _Cfg(
        config.playW,
        config.playH,
        config.minimumCueElevationDeg,
        config.clothSlidingFriction,
        config.clothRollingResistance,
      );
}

class _CornerFrame {
  final String name;
  final Vec2 origin;
  final double xInward;
  final double yInward;

  const _CornerFrame(this.name, this.origin, this.xInward, this.yInward);

  Vec2 toWorld(Vec2 local) =>
      Vec2(origin.x + xInward * local.x, origin.y + yInward * local.y);

  Vec2 toLocal(Vec2 world) =>
      Vec2((world.x - origin.x) * xInward, (world.y - origin.y) * yInward);
}

class _PocketSegment {
  final _CornerFrame corner;
  final String surface;
  final Vec2 start;
  final Vec2 end;

  const _PocketSegment(this.corner, this.surface, this.start, this.end);
}

// ── Coordinate helpers ───────────────────────────────────────────────────────

Vec2 _relToMeters(Vec2 rel, _Cfg cfg) => Vec2(
      (rel.x - _relX0) / (_relX1 - _relX0) * cfg.W,
      (rel.y - _relY0) / (_relY1 - _relY0) * cfg.H,
    );

Vec2 _metersToRel(Vec2 m, _Cfg cfg) => Vec2(
      m.x / cfg.W * (_relX1 - _relX0) + _relX0,
      m.y / cfg.H * (_relY1 - _relY0) + _relY0,
    );

/// Ghost ball centre in metres (where cue ball is at moment of first contact).
Vec2? computeGhostBallMeters(
  Vec2 cuePosRel,
  Vec2 aimDirPhys,
  List<PhysicsBall> balls,
  String cueBallId,
  PhysicsTableConfig tableConfig,
) {
  final cfg = _Cfg.from(tableConfig);
  final cueM = _relToMeters(cuePosRel, cfg);
  double? minT;
  for (final b in balls) {
    if (b.id == cueBallId || b.pocketed) continue;
    final objM = _relToMeters(b.position, cfg);
    final delta = cueM - objM;
    final dot = delta.x * aimDirPhys.x + delta.y * aimDirPhys.y;
    final disc = dot * dot -
        (delta.x * delta.x + delta.y * delta.y) +
        (2 * _R) * (2 * _R);
    if (disc < 0) continue;
    final t = -dot - math.sqrt(disc);
    if (t > 0 && (minT == null || t < minT)) minT = t;
  }
  if (minT == null) return null;
  final ghostM = cueM + aimDirPhys * minT;
  if (ghostM.x < _R ||
      ghostM.x > cfg.W - _R ||
      ghostM.y < _R ||
      ghostM.y > cfg.H - _R) {
    return null;
  }
  return ghostM;
}

// ── Public types ─────────────────────────────────────────────────────────────

/// Cue deflection model used to compute squirt angle and effective spin offset.
///
/// [lowDeflection] and [standard] use the analytical TP A.31 end-mass model
/// with calibrated end masses (~5.4 g and ~7.8 g respectively, chosen to
/// reproduce Dr. Dave's representative max-offset squirt angles of ~1.8° and
/// ~2.5°). [playersMeasured] and [predatorZMeasured] use masses inferred from
/// TP B.1 video-measured squirt data.
enum CueType {
  /// Low-deflection shaft (~5.4 g effective end mass, ~1.8° max squirt).
  lowDeflection,

  /// Standard shaft (~7.8 g effective end mass, ~2.5° max squirt).
  standard,

  /// Players cue inferred from TP B.1 measurements (mr = 20.151).
  playersMeasured,

  /// Predator Z cue inferred from TP B.1 measurements (mr = 29.158).
  predatorZMeasured,
}

double _cueEffectiveEndMassKg(CueType cueType) => switch (cueType) {
      CueType.lowDeflection => _endMassLowDeflectionKg,
      CueType.standard => _endMassStandardKg,
      // TP B.1 infers these end masses from measured squirt at a 0.51-inch tip
      // offset: Players regular cue mr=20.151; Predator Z mr=29.158.
      CueType.playersMeasured => _cueBallMassKg / 20.151,
      CueType.predatorZMeasured => _cueBallMassKg / 29.158,
    };

/// Velocity-dependent ball–ball friction coefficient.
///
/// Alciatore's curve: μ = 0.009951 + 0.108 · exp(−1.088 · v), where v is
/// the relative tangential slip speed in m/s. At low speed (v ≈ 0) μ ≈ 0.118;
/// at high speed it asymptotes near μ ≈ 0.010. [Dr. Dave TP A.14]
double ballBallFrictionForSlip(double relativeSlipSpeedMs) {
  final speed = math.max(0.0, relativeSlipSpeedMs);
  return 0.009951 + 0.108 * math.exp(-1.088 * speed);
}

/// Initial cue-ball squirt angle from Dr. Dave's TP A.31 effective-endmass
/// model. [hitX] is signed lateral tip offset in ball radii.
double cueSquirtAngleRad(double hitX, CueType cueType) {
  final offset = hitX.abs().clamp(0.0, _miscueLimit);
  if (offset == 0) return 0.0;
  final effectiveEndMass = _cueEffectiveEndMassKg(cueType);
  final tangent = (2.5 * offset * math.sqrt(1.0 - offset * offset)) /
      (1.0 + _cueBallMassKg / effectiveEndMass + 2.5 * (1.0 - offset * offset));
  // The simulator's table y axis points down, opposite the right-handed
  // physical y axis used by the published cue-deflection convention.
  return -math.atan(tangent) * hitX.sign;
}

/// Effective lateral tip offset after the cue's squirt rotates its impact
/// impulse away from the cue axis (Dr. Dave TP B.7). Spin-producing leverage
/// is the perpendicular distance from the ball centre to the impulse line.
double cueEffectiveLateralOffsetR(double hitX, CueType cueType) {
  final offset = hitX.abs().clamp(0.0, _miscueLimit);
  if (offset == 0) return 0.0;
  final squirt = cueSquirtAngleRad(hitX, cueType).abs();
  final effectiveOffset = math.sin(math.asin(offset) - squirt);
  return hitX.sign * effectiveOffset;
}

/// Parameters describing a single cue shot.
///
/// All tip-offset fields follow the TP A.12 / TP 2.1 normalised convention:
/// zero is ball centre, ±1 is the equator on that axis. The combined radial
/// offset `sqrt(hitX² + hitY²)` is clamped to the miscue limit
/// ([maximumNoSlipCueTipOffsetR]) before the simulation applies it.
///
/// [aimDir] is the unit vector in table-relative coordinates pointing from
/// the cue ball toward the intended target. [speedMs] is the cue-ball speed
/// at the cloth immediately after cue impact (not the cue-stick speed).
class ShotParams {
  /// Normalised aim direction on the table plane.
  final Vec2 aimDir;

  /// Cue-ball launch speed in m/s.
  final double speedMs;

  /// Signed lateral tip offset in ball radii (−1 = far left, +1 = far right).
  final double hitX;

  /// Signed vertical tip offset in ball radii (−1 = full draw, +1 = full follow).
  final double hitY;

  final CueType cueType;
  final double? elevationDeg;
  final double cueTipDiameterM;
  final double cueTipCurvatureRadiusM;

  // Research mode: suppresses cue-induced vertical launch to compare massé
  // cloth trajectories against Dr. Dave TP A.19's continuous-contact model.
  // Ordinary shots always retain the hop model.
  final bool continuousClothContact;

  const ShotParams({
    required this.aimDir,
    this.speedMs = 2.5,
    this.hitX = 0.0,
    this.hitY = 0.0,
    this.cueType = CueType.lowDeflection,
    this.elevationDeg,
    this.cueTipDiameterM = referenceCueTipDiameterM,
    this.cueTipCurvatureRadiusM = referenceCueTipCurvatureRadiusM,
    this.continuousClothContact = false,
  });
}

/// Rail geometry and friction parameters for the Mathavan et al. cushion model.
///
/// The defaults model a standard pool table cushion. [standardPool] and
/// [diamondRedLabel] are pre-calibrated constants; individual fields can be
/// overridden for sensitivity studies.
///
/// The Mathavan model is documented in:
/// S. Mathavan, M. R. Jackson, and R. M. Parkin, "Numerical simulations of
/// the frictional interactions during pool/snooker ball–cushion contacts,"
/// *Proc. IMechE Part C*, 2010.
class CushionProfile {
  final double noseHeightMm;
  final double restitution;
  final double airborneRestitution;
  final double pocketFacingRestitution;
  final double slidingFriction;
  final double tableImpactFriction;

  @Deprecated('Low-speed grip suppression is no longer modeled')
  final double lowSpeedFriction;

  final double highSpeedFriction;
  final double peakGripSpeed;
  final double highSpeedReference;
  final double sidespinGrip;
  final double tangentialSpinTransfer;
  final double tangentialRestitution;
  final double highSpeedTangentialRestitution;
  final double impactDurationMs;

  const CushionProfile({
    this.noseHeightMm = 36.3,
    // Mathavan et al.'s energetic COR = 0.98 is fitted from snooker-table
    // measurements, not a verified pool-table value. Their 0.818 effective
    // speed ratio is for one rolling, perpendicular snooker-ball condition;
    // neither number should be confused with Dr. Dave TP B.6's approximate
    // 0.7 effective pool rebound ratio.
    this.restitution = 0.98,
    // The grounded model assumes continuous cloth contact. Until an airborne
    // impact model is calibrated, retain the legacy effective-COR parameter.
    this.airborneRestitution = 0.82,
    // Pocket facings are angled, rounded rubber rather than the straight rail
    // nose represented by the Mathavan profile; use the empirical effective
    // pool-cushion range until facing-specific data are available.
    this.pocketFacingRestitution = 0.82,
    this.slidingFriction = 0.14,
    this.tableImpactFriction = 0.20,
    this.lowSpeedFriction = 0.14,
    this.highSpeedFriction = 0.13,
    this.peakGripSpeed = 2.60,
    this.highSpeedReference = 4.00,
    this.sidespinGrip = 1.50,
    this.tangentialSpinTransfer = 0.55,
    this.tangentialRestitution = 0.44,
    this.highSpeedTangentialRestitution = 0.60,
    this.impactDurationMs = 7.0,
  });

  static const standardPool = CushionProfile();

  static const diamondRedLabel = CushionProfile(
    noseHeightMm: 32.0,
    restitution: 0.98,
    airborneRestitution: 0.85,
    slidingFriction: 0.145,
    tableImpactFriction: 0.20,
    lowSpeedFriction: 0.065,
    highSpeedFriction: 0.125,
    peakGripSpeed: 2.50,
    highSpeedReference: 3.80,
    sidespinGrip: 1.65,
    tangentialSpinTransfer: 0.55,
    tangentialRestitution: 0.50,
    highSpeedTangentialRestitution: 0.62,
    impactDurationMs: 7.5,
  );

  double gripAtApproach(double approachSpeedMs) =>
      _cushionGrip(this, approachSpeedMs);
}

// ── Internal ball ────────────────────────────────────────────────────────────

class _Ball {
  final String id;
  Vec3 pos3;
  Vec3 vel3;
  Vec3 angularVelocity;
  bool active;
  bool pocketed;
  Vec2? pocketPosition;
  Set<String> activePocketSurfaces = <String>{};

  _Ball(this.id, Vec2 pos, {this.pocketed = false})
      : pos3 = Vec3(pos.x, pos.y, _R),
        vel3 = Vec3.zero,
        angularVelocity = Vec3.zero,
        active = !pocketed;

  Vec2 get pos => Vec2(pos3.x, pos3.y);

  set pos(Vec2 value) => pos3 = Vec3(value.x, value.y, pos3.z);

  Vec2 get vel => Vec2(vel3.x, vel3.y);

  set vel(Vec2 value) => vel3 = Vec3(value.x, value.y, vel3.z);

  Vec2 get spin => Vec2(_R * angularVelocity.y, -_R * angularVelocity.x);

  set spin(Vec2 value) =>
      angularVelocity = Vec3(-value.y / _R, value.x / _R, angularVelocity.z);

  double get spinZ => angularVelocity.z;

  set spinZ(double value) =>
      angularVelocity = Vec3(angularVelocity.x, angularVelocity.y, value);

  bool get isMoving =>
      active && (vel3.length > 4e-4 || angularVelocity.length > 0.02);
}

// ── Simulation ───────────────────────────────────────────────────────────────

/// Runs a complete billiards shot from cue-ball impact to final rest.
///
/// Input [balls] use relative table coordinates (see [PhysicsTableConfig]).
/// Output positions in [PhysicsResult] use the same coordinate space.
///
/// The simulation integrates at 1 ms steps and records path points every 8 ms.
/// [forPreview] uses a coarser 3 ms step and 40 ms recording interval for
/// real-time aim preview; it is visually indistinguishable for aiming but
/// less accurate for validation.
///
/// [ballBallFrictionScale] multiplies Alciatore's friction curve uniformly —
/// useful for ball-condition sensitivity studies (Dr. Dave TP A.17).
PhysicsResult simulate({
  required List<PhysicsBall> balls,
  required String cueBallId,
  required ShotParams shot,
  required PhysicsTableConfig tableConfig,
  CushionProfile cushion = CushionProfile.standardPool,
  int shotIndex = 0,
  bool forPreview = false,
  double? maxTimeSeconds,
  double ballBallFrictionScale = 1.0,
}) {
  if (!ballBallFrictionScale.isFinite || ballBallFrictionScale < 0) {
    throw ArgumentError.value(
      ballBallFrictionScale,
      'ballBallFrictionScale',
      'must be finite and non-negative',
    );
  }
  final dt = forPreview ? 0.003 : _dt;
  final recordEvery = forPreview ? 0.040 : _recordEvery;
  final maxTime = maxTimeSeconds ?? (forPreview ? 6.0 : _maxTime);
  final cfg = _Cfg.from(tableConfig);

  final simBalls = balls
      .map(
        (b) => _Ball(b.id, _relToMeters(b.position, cfg), pocketed: b.pocketed),
      )
      .toList();

  final cue = simBalls.firstWhere((b) => b.id == cueBallId);
  final geomElev = _computeElevationDeg(
    cue.pos,
    shot.aimDir,
    simBalls,
    cueBallId,
    cfg,
  );
  final elevDeg = shot.elevationDeg != null
      ? math.max(geomElev, shot.elevationDeg!)
      : geomElev;
  _initCueBall(cue, shot, elevDeg);

  final pathPoints = <String, List<Vec2>>{};
  for (final b in simBalls) {
    pathPoints[b.id] = [_metersToRel(b.pos, cfg)];
  }
  // Animation frames: all non-pocketed balls at each _recordEvery timestep.
  final frames = <Map<String, Vec2>>[];
  frames.add({
    for (final b in simBalls)
      if (!b.pocketed) b.id: _metersToRel(b.pos, cfg),
  });
  final peakHeights = <String, double>{
    for (final b in simBalls) b.id: b.pos3.z,
  };
  final tableBounceCounts = <String, int>{for (final b in simBalls) b.id: 0};
  final tableBounceDistancesMeters = <String, List<double>>{
    for (final b in simBalls) b.id: <double>[],
  };
  final tableBounceSpeedsMs = <String, List<double>>{
    for (final b in simBalls) b.id: <double>[],
  };
  final tableBounceSpinRps = <String, List<double>>{
    for (final b in simBalls) b.id: <double>[],
  };
  final horizontalDistanceMeters = <String, double>{
    for (final b in simBalls) b.id: 0.0,
  };
  final peakSpinZ = <String, double>{
    for (final b in simBalls) b.id: b.spinZ.abs(),
  };
  final peakSpinRpm = <String, double>{
    for (final b in simBalls)
      b.id: b.angularVelocity.length * _radiansPerSecondToRpm,
  };
  final sideSpinRatioAtRoll = <String, double>{};

  double t = 0.0;
  double nextRecord = recordEvery;
  int? cueContactStep;
  Vec2? ghostBallMeters;
  int step = 0;
  var cushionContacts = 0;
  final cushionContactDetails = <CushionContact>[];
  final pocketContactDetails = <PocketContact>[];
  final ballBallContactDetails = <BallBallContact>[];

  while (t < maxTime) {
    t += dt;
    step++;

    final previousPositions = <String, Vec3>{
      for (final b in simBalls) b.id: b.pos3,
    };
    final previousVelocities = <String, Vec3>{
      for (final b in simBalls) b.id: b.vel3,
    };
    final previousAngularVelocities = <String, Vec3>{
      for (final b in simBalls) b.id: b.angularVelocity,
    };
    final previousBounceCounts = Map<String, int>.of(tableBounceCounts);
    _applyFriction(simBalls, tableBounceCounts, cfg, sideSpinRatioAtRoll);
    final hitInfo = _resolveCollisions(
      simBalls,
      cueBallId,
      cfg,
      previousPositions: previousPositions,
      previousVelocities: previousVelocities,
      previousAngularVelocities: previousAngularVelocities,
      ballBallFrictionScale: ballBallFrictionScale,
      timeSec: t,
      timeStepSec: dt,
      contactDetails: ballBallContactDetails,
    );
    if (hitInfo != null && cueContactStep == null) {
      cueContactStep = step;
      ghostBallMeters = hitInfo;
    }
    for (final b in simBalls) {
      final previous = previousPositions[b.id]!;
      final dx = b.pos3.x - previous.x;
      final dy = b.pos3.y - previous.y;
      horizontalDistanceMeters[b.id] =
          horizontalDistanceMeters[b.id]! + math.sqrt(dx * dx + dy * dy);
      if (tableBounceCounts[b.id]! > previousBounceCounts[b.id]!) {
        tableBounceDistancesMeters[b.id]!.add(horizontalDistanceMeters[b.id]!);
        tableBounceSpeedsMs[b.id]!.add(b.vel3.length);
        tableBounceSpinRps[b.id]!.add(b.angularVelocity.length / (2 * math.pi));
      }
    }
    final cushionContactIds = <String>{};
    cushionContacts += _resolveCushions(
      simBalls,
      cfg,
      cushion,
      contactIds: cushionContactIds,
      contactDetails: cushionContactDetails,
      timeSec: t,
    );
    for (final b in simBalls) {
      _limitAngularSpeed(b);
    }
    for (final id in cushionContactIds) {
      final ball = simBalls.firstWhere((value) => value.id == id);
      final point = _metersToRel(ball.pos, cfg);
      final points = pathPoints[id]!;
      if (points.isEmpty || (points.last - point).distance > 1e-7) {
        points.add(point);
      }
    }
    final pocketContactIds = <String>{};
    _resolvePocketGeometry(
      simBalls,
      cfg,
      cushion,
      contactIds: pocketContactIds,
      contactDetails: pocketContactDetails,
      timeSec: t,
    );
    for (final id in pocketContactIds) {
      final ball = simBalls.firstWhere((value) => value.id == id);
      final point = _metersToRel(ball.pos, cfg);
      final points = pathPoints[id]!;
      if (points.isEmpty || (points.last - point).distance > 1e-7) {
        points.add(point);
      }
    }
    _checkPockets(simBalls, cfg);
    for (final b in simBalls) {
      if (b.pos3.z > (peakHeights[b.id] ?? _R)) peakHeights[b.id] = b.pos3.z;
      if (b.spinZ.abs() > (peakSpinZ[b.id] ?? 0.0)) {
        peakSpinZ[b.id] = b.spinZ.abs();
      }
      final spinRpm = b.angularVelocity.length * _radiansPerSecondToRpm;
      if (spinRpm > (peakSpinRpm[b.id] ?? 0.0)) {
        peakSpinRpm[b.id] = spinRpm;
      }
    }

    if (t >= nextRecord) {
      nextRecord += recordEvery;
      for (final b in simBalls) {
        if (b.isMoving) pathPoints[b.id]!.add(_metersToRel(b.pos, cfg));
      }
      frames.add({
        for (final b in simBalls)
          if (!b.pocketed) b.id: _metersToRel(b.pos, cfg),
      });
    }

    if (simBalls.every((b) => !b.isMoving)) break;
  }

  // Preserve terminal state in the replay. A ball can be pocketed between
  // samples, then stop the simulation before the next regular frame is due.
  // Without this frame the animation freezes on the last pre-drop position.
  final finalFrame = {
    for (final b in simBalls)
      if (!b.pocketed) b.id: _metersToRel(b.pos, cfg),
  };
  final lastFrame = frames.last;
  final frameChanged = lastFrame.length != finalFrame.length ||
      finalFrame.entries.any((entry) => lastFrame[entry.key] != entry.value);
  if (frameChanged) frames.add(finalFrame);

  final endPositions = <String, Vec2?>{};
  final pocketPositions = <String, Vec2>{};
  final endHeights = <String, double>{};
  final cornerShelfBalls = <String>{};
  for (final b in simBalls) {
    endPositions[b.id] = b.pocketed ? null : _metersToRel(b.pos, cfg);
    endHeights[b.id] = b.pos3.z;
    if (!b.pocketed && _onCornerShelfOrBetweenJaws(b.pos, cfg)) {
      cornerShelfBalls.add(b.id);
    }
    final points = pathPoints[b.id]!;
    if (b.pocketed && b.pocketPosition != null) {
      pocketPositions[b.id] = _metersToRel(b.pocketPosition!, cfg);
      final pocketRel = pocketPositions[b.id]!;
      if (points.isEmpty || (points.last - pocketRel).distance > 1e-5) {
        points.add(pocketRel);
      }
    } else {
      final endRel = _metersToRel(b.pos, cfg);
      if (points.isEmpty || (points.last - endRel).distance > 1e-5) {
        points.add(endRel);
      }
    }
  }

  final ghostBallRel =
      ghostBallMeters != null ? _metersToRel(ghostBallMeters, cfg) : null;

  final cueContactIdx = cueContactStep != null
      ? (cueContactStep * _dt / _recordEvery).floor().clamp(
            0,
            pathPoints[cueBallId]!.length - 1,
          )
      : null;

  final paths = <PhysicsPath>[];
  for (final entry in pathPoints.entries) {
    if (entry.value.length < 2) continue;
    final isCue = entry.key == cueBallId;
    if (isCue && cueContactIdx != null && cueContactIdx > 0) {
      paths.add(
        PhysicsPath(
          ballId: entry.key,
          points: entry.value.sublist(0, cueContactIdx + 1),
          style: PhysicsPathStyle.cueBefore,
          shotIndex: shotIndex,
        ),
      );
      if (cueContactIdx < entry.value.length - 1) {
        paths.add(
          PhysicsPath(
            ballId: entry.key,
            points: entry.value.sublist(cueContactIdx),
            style: PhysicsPathStyle.cueAfter,
            shotIndex: shotIndex,
          ),
        );
      }
    } else {
      paths.add(
        PhysicsPath(
          ballId: entry.key,
          points: entry.value,
          style:
              isCue ? PhysicsPathStyle.cueBefore : PhysicsPathStyle.objectBall,
          shotIndex: shotIndex,
        ),
      );
    }
  }

  return PhysicsResult(
    paths: paths,
    endPositions: endPositions,
    pocketPositions: pocketPositions,
    endHeights: endHeights,
    peakHeights: peakHeights,
    tableBounceCounts: tableBounceCounts,
    tableBounceDistancesMeters: tableBounceDistancesMeters,
    tableBounceSpeedsMs: tableBounceSpeedsMs,
    tableBounceSpinRps: tableBounceSpinRps,
    peakSpinZ: peakSpinZ,
    peakSpinRpm: peakSpinRpm,
    finalLinearVelocities: {for (final b in simBalls) b.id: b.vel3},
    sideSpinRatioAtRoll: sideSpinRatioAtRoll,
    cushionContacts: cushionContacts,
    cushionContactDetails: cushionContactDetails,
    pocketContactDetails: pocketContactDetails,
    ballBallContactDetails: ballBallContactDetails,
    cornerShelfBalls: cornerShelfBalls,
    ghostBallRel: ghostBallRel,
    cueElevationDeg: elevDeg,
    frames: frames,
  );
}

// ── Elevation computation ─────────────────────────────────────────────────────

/// Public wrapper: minimum cue elevation (degrees) required by the table config
/// and to clear the rail or balls in the bridge zone behind the cue ball.
double computeElevationDeg(
  Vec2 cuePosRel,
  Vec2 aimDir,
  List<PhysicsBall> balls,
  String cueBallId,
  PhysicsTableConfig tableConfig,
) {
  final cfg = _Cfg.from(tableConfig);
  final cueM = _relToMeters(cuePosRel, cfg);
  final simBalls = balls
      .map(
        (b) => _Ball(b.id, _relToMeters(b.position, cfg), pocketed: b.pocketed),
      )
      .toList();
  return _computeElevationDeg(cueM, aimDir, simBalls, cueBallId, cfg);
}

double _computeElevationDeg(
  Vec2 cbPos,
  Vec2 aimDir,
  List<_Ball> balls,
  String cueBallId,
  _Cfg cfg,
) {
  // TP A.3 uses a 1 11/32 in rail and a 0.722 in cue diameter. Its worked
  // 24 in head-spot clearance example is a useful check on this geometry.
  const double hRail = 1.34375 * inchesToMeters;
  const double lBridge = 0.35;
  const double rCue = 0.722 / 2 * inchesToMeters;
  const double cueLength = 57 * inchesToMeters;

  double elevDeg = cfg.minimumCueElevationDeg;

  final bdx = -aimDir.x;
  final bdy = -aimDir.y;
  double tRail = double.infinity;
  if (bdx < -1e-9) tRail = math.min(tRail, cbPos.x / (-bdx));
  if (bdx > 1e-9) tRail = math.min(tRail, (cfg.W - cbPos.x) / bdx);
  if (bdy < -1e-9) tRail = math.min(tRail, cbPos.y / (-bdy));
  if (bdy > 1e-9) tRail = math.min(tRail, (cfg.H - cbPos.y) / bdy);

  if (tRail > 1e-4 && tRail < cueLength) {
    final clearanceDeg = _cueRailClearanceElevationDeg(tRail, hRail, rCue);
    elevDeg = math.max(elevDeg, clearanceDeg);
  }

  for (final ball in balls) {
    if (ball.id == cueBallId || !ball.active) continue;
    final diff = ball.pos - cbPos;

    final tBack = -(diff.x * aimDir.x + diff.y * aimDir.y);
    if (tBack < _R * 2 || tBack > lBridge + _R) continue;

    final axial = diff.x * aimDir.x + diff.y * aimDir.y;
    final perpX = diff.x - axial * aimDir.x;
    final perpY = diff.y - axial * aimDir.y;
    final lateral = math.sqrt(perpX * perpX + perpY * perpY);
    if (lateral >= _R + rCue) continue;

    final hBallTop =
        _R + (lateral < _R ? math.sqrt(_R * _R - lateral * lateral) : 0.0);
    final tanA = (hBallTop + rCue - _R) / tBack;
    if (tanA > 0) elevDeg = math.max(elevDeg, math.atan(tanA) * 180 / math.pi);
  }

  return elevDeg;
}

// Dr. Dave TP A.3's cue/rail cross-section geometry. `distance` is the
// horizontal distance from the cue-ball center to the rail plane behind it.
double _cueRailClearanceElevationDeg(
  double distance,
  double railHeight,
  double cueRadius,
) {
  double clearanceAt(double angle) {
    final alongCue = (distance - cueRadius * math.sin(angle)) / math.cos(angle);
    return _R +
        alongCue * math.sin(angle) -
        railHeight -
        cueRadius * math.cos(angle);
  }

  if (clearanceAt(0) >= 0) return 0;
  var low = 0.0;
  var high = math.pi / 2 - 1e-6;
  for (var i = 0; i < 60; i++) {
    final mid = (low + high) / 2;
    if (clearanceAt(mid) >= 0) {
      high = mid;
    } else {
      low = mid;
    }
  }
  return high * 180 / math.pi;
}

// ── Cue ball initialisation ───────────────────────────────────────────────────

void _initCueBall(_Ball cue, ShotParams shot, double elevationDeg) {
  // Enforce miscue limit: scale hit point down if outside the reachable contact zone.
  double hitX = shot.hitX;
  double hitY = shot.hitY;
  final hitMag = math.sqrt(hitX * hitX + hitY * hitY);
  if (hitMag > _miscueLimit) {
    final scale = _miscueLimit / hitMag;
    hitX *= scale;
    hitY *= scale;
  }
  // TP A.22 adds a cue/table clearance boundary on the draw side that is
  // independent of the radial TP 2.1 no-slip/miscue limit. The reference cue
  // profile is a 12.5 mm nickel-radius tip; elevated strokes gain clearance.
  hitY = constrainCueDrawOffsetR(
    requestedOffsetR: hitY,
    cueElevationDeg: elevationDeg,
    maximumMiscueOffsetR: _miscueLimit,
    tipDiameterM: shot.cueTipDiameterM,
    tipCurvatureRadiusM: shot.cueTipCurvatureRadiusM,
  );

  final squirtAngle = cueSquirtAngleRad(hitX, shot.cueType);
  final ca = math.cos(squirtAngle);
  final sa = math.sin(squirtAngle);
  final sqDir = Vec2(
    shot.aimDir.x * ca - shot.aimDir.y * sa,
    shot.aimDir.x * sa + shot.aimDir.y * ca,
  );

  final elevRad = elevationDeg * math.pi / 180;
  final cosElev = math.cos(elevRad);
  // Treat the speed control as the launch-speed magnitude. Cue elevation
  // resolves that vector into horizontal travel and downward table impact;
  // the table, rather than the cue, generates any upward hop.
  final horizontalSpeed = shot.speedMs * cosElev;
  final downwardSpeed =
      shot.continuousClothContact ? 0.0 : shot.speedMs * math.sin(elevRad);
  cue.vel3 = Vec3(
    sqDir.x * horizontalSpeed,
    sqDir.y * horizontalSpeed,
    -downwardSpeed,
  );

  final cueStickVel = shot.speedMs;
  final effectiveHitX = cueEffectiveLateralOffsetR(hitX, shot.cueType);

  // TP A.19's tip-offset equations (also used by B.10's draw-hop examples)
  // compute longitudinal spin from cue speed and normalized contact offset,
  // independent of cue elevation. Elevation changes the launch vector and
  // subsequent bounce sequence; it is not a cosine scale on delivered spin.
  cue.spin = sqDir * (cueStickVel * hitY * _spinYFactor);
  cue.spinZ = -effectiveHitX * (cueStickVel / _R) * _spinZFactor * cosElev;

  final sinElev = math.sin(elevRad);
  if (sinElev > 1e-3 && hitX.abs() > 1e-3) {
    // Positive hitX is right english, so its masse component must point to
    // the shot's right-hand side (in screen/table coordinates).
    final perp = Vec2(-sqDir.y, sqDir.x);
    // The lateral tip offset crossed with the downward cue impulse creates
    // massé spin. For a solid sphere I=2/5 mR², so Rω/V = (5/2)(b/R)
    // times the vertical fraction of the cue impulse.
    final lateral = hitX * cueStickVel * 2.5 * sinElev;
    cue.spin = cue.spin + perp * lateral;
  }
}

// ── Physics ───────────────────────────────────────────────────────────────────

void _applyFriction(
  List<_Ball> balls,
  Map<String, int> tableBounceCounts,
  _Cfg cfg,
  Map<String, double> sideSpinRatioAtRoll,
) {
  for (final b in balls) {
    if (!b.active) continue;

    final airborne = b.pos3.z > _R + 1e-8 || b.vel3.z > 0;
    if (airborne) {
      b.vel3 += const Vec3(0, 0, -_g * _dt);
    }

    // Dr. Dave's high-speed-video analysis shows that a ball retains its spin
    // while airborne: cloth friction acts during table contact, not during a
    // hop. Apply the existing sliding/rolling model only while grounded.
    if (!airborne) {
      final vc = b.vel - b.spin;
      final vcSpeed = vc.distance;

      if (vcSpeed > 4e-4) {
        final vcDir = vc / vcSpeed;
        final slipAccel = cfg.clothSlidingFriction * _g;
        final slipStep = 3.5 * slipAccel * _dt;
        if (vcSpeed <= slipStep) {
          final rolling = b.vel - vcDir * (vcSpeed / 3.5);
          b.vel = rolling;
          b.spin = rolling;
          sideSpinRatioAtRoll.putIfAbsent(
            b.id,
            () => b.vel.distance > 1e-9
                ? b.spinZ.abs() * _R / b.vel.distance
                : 0.0,
          );
        } else {
          b.vel = b.vel - vcDir * (slipAccel * _dt);
          b.spin = b.spin + vcDir * (2.5 * slipAccel * _dt);
        }
      } else {
        final speed = b.vel.distance;
        if (speed > 4e-4) {
          // Dr. Dave TP B.2: the off-centre normal force that produces
          // rolling resistance also creates a small lateral force when the
          // ball has sidespin. Integrate its turn rate only in the rolling
          // regime; sliding and airborne motion use their own contact laws.
          // Solve TP B.2's force-balance relation:
          // sin(theta) - muR*cos(theta) = (2/5)*muR.
          final turnNormalAngle = math.atan(cfg.clothRollingResistance) +
              math.asin(
                (2.0 / 5.0) *
                    cfg.clothRollingResistance /
                    math.sqrt(
                      1.0 +
                          cfg.clothRollingResistance *
                              cfg.clothRollingResistance,
                    ),
              );
          final turnRate = (2.0 / 5.0) *
              _R *
              spinZDecayRateRadPerSec2 *
              math.sin(turnNormalAngle) /
              (speed * (2.0 / 5.0 + math.cos(turnNormalAngle)));
          final spinDuration = b.spinZ.abs() > 0.02
              ? math.min(_dt, b.spinZ.abs() / spinZDecayRateRadPerSec2)
              : 0.0;
          final turnAngle = -b.spinZ.sign * turnRate * spinDuration;
          final cosTurn = math.cos(turnAngle);
          final sinTurn = math.sin(turnAngle);
          final turnedVelocity = Vec2(
            b.vel.x * cosTurn - b.vel.y * sinTurn,
            b.vel.x * sinTurn + b.vel.y * cosTurn,
          );
          b.spin = turnedVelocity;
          b.vel = turnedVelocity -
              turnedVelocity.normalized *
                  (cfg.clothRollingResistance * _g * _dt);
        } else {
          b.vel = Vec2.zero;
          b.spin = Vec2.zero;
        }
      }

      if (b.spinZ.abs() > 0.02) {
        final decay = spinZDecayRateRadPerSec2 * _dt;
        b.spinZ = b.spinZ.abs() <= decay ? 0.0 : b.spinZ - decay * b.spinZ.sign;
      } else {
        b.spinZ = 0.0;
      }

      if (b.vel.distance < 3e-4 &&
          b.spin.distance < 3e-4 &&
          b.spinZ.abs() < 0.02) {
        b.vel = Vec2.zero;
        b.spin = Vec2.zero;
        b.spinZ = 0.0;
      }
    }

    b.pos3 += b.vel3 * _dt;
    if (b.pos3.z < _R) {
      b.pos3 = Vec3(b.pos3.x, b.pos3.y, _R);
      if (b.vel3.z < 0) {
        _resolveTableLanding(b, cfg.clothSlidingFriction);
        tableBounceCounts[b.id] = (tableBounceCounts[b.id] ?? 0) + 1;
      }
    }
  }
}

void _resolveTableLanding(_Ball ball, double clothSlidingFriction) {
  final impactSpeed = -ball.vel3.z;
  final normalImpulse = (1 + _eTable) * impactSpeed;
  final normalVelocity = impactSpeed < 0.08 ? 0.0 : _eTable * impactSpeed;

  // Coulomb-limited tangential impulse at the cloth. For a solid sphere,
  // effective tangential mass is m + I/R² = 3.5m. This couples hop landings
  // back into both planar velocity and spin instead of teleporting z to rest.
  final slipX = ball.vel3.x - ball.angularVelocity.y * _R;
  final slipY = ball.vel3.y + ball.angularVelocity.x * _R;
  var impulseX = -slipX / (1 + 1 / _ballInertia);
  var impulseY = -slipY / (1 + 1 / _ballInertia);
  final tangentImpulse = math.sqrt(impulseX * impulseX + impulseY * impulseY);
  final frictionLimit = clothSlidingFriction * normalImpulse;
  if (tangentImpulse > frictionLimit && tangentImpulse > 0) {
    final scale = frictionLimit / tangentImpulse;
    impulseX *= scale;
    impulseY *= scale;
  }

  ball.vel3 = Vec3(
    ball.vel3.x + impulseX,
    ball.vel3.y + impulseY,
    normalVelocity,
  );
  // r=(0,0,-R); angular impulse is r × J / I.
  ball.angularVelocity += Vec3(
    impulseY / (_ballInertia * _R),
    -impulseX / (_ballInertia * _R),
    0,
  );
}

class _HertzChainResolution {
  final Set<String> ballIds;
  final Vec2? cueContact;

  const _HertzChainResolution(this.ballIds, this.cueContact);
}

/// Handles initially touching, collinear, spin-free chains with a finite
/// Hertz normal-contact interval. Other contacts stay on the general 3-D
/// impulse path below, where tangential friction and spin transfer are modeled.
_HertzChainResolution _resolveHertzChains(
  List<_Ball> balls,
  String cueBallId,
  Map<String, Vec3>? previousPositions,
  Map<String, Vec3>? previousVelocities,
  Map<String, Vec3>? previousAngularVelocities,
  double durationSeconds,
) {
  if (balls.length < 3 ||
      previousPositions == null ||
      previousVelocities == null ||
      previousAngularVelocities == null ||
      durationSeconds <= 0) {
    return const _HertzChainResolution(<String>{}, null);
  }

  final unvisited = <int>{
    for (var i = 0; i < balls.length; i++)
      if (balls[i].active && previousPositions.containsKey(balls[i].id)) i,
  };
  final handledIds = <String>{};
  Vec2? cueContact;

  while (unvisited.isNotEmpty) {
    final seed = unvisited.first;
    unvisited.remove(seed);
    final cluster = <int>[seed];
    final queue = <int>[seed];
    while (queue.isNotEmpty) {
      final i = queue.removeLast();
      final posI = previousPositions[balls[i].id]!;
      for (final j in unvisited.toList()) {
        final posJ = previousPositions[balls[j].id]!;
        final touching = (posJ - posI).length <= 2 * _R + 1e-6;
        final willContact = _ballBallTimeOfImpactFraction(
          posI,
          posJ,
          balls[i].pos3,
          balls[j].pos3,
        );
        if (touching || willContact != null) {
          unvisited.remove(j);
          cluster.add(j);
          queue.add(j);
        }
      }
    }
    if (cluster.length < 3) continue;

    var firstIndex = cluster.first;
    var lastIndex = cluster.last;
    var greatestDistanceSquared = 0.0;
    for (var i = 0; i < cluster.length; i++) {
      for (var j = i + 1; j < cluster.length; j++) {
        final delta = previousPositions[balls[cluster[j]].id]! -
            previousPositions[balls[cluster[i]].id]!;
        if (delta.lengthSquared > greatestDistanceSquared) {
          greatestDistanceSquared = delta.lengthSquared;
          firstIndex = cluster[i];
          lastIndex = cluster[j];
        }
      }
    }
    if (greatestDistanceSquared < 1e-12) continue;
    final axis = (previousPositions[balls[lastIndex].id]! -
            previousPositions[balls[firstIndex].id]!)
        .normalized();
    final ordered = cluster.toList()
      ..sort(
        (a, b) => previousPositions[balls[a].id]!
            .dot(axis)
            .compareTo(previousPositions[balls[b].id]!.dot(axis)),
      );
    final origin = previousPositions[balls[ordered.first].id]!;
    var collinear = true;
    var spinFree = true;
    for (final index in ordered) {
      final ball = balls[index];
      final positionDelta = previousPositions[ball.id]! - origin;
      final positionResidual = positionDelta - axis * positionDelta.dot(axis);
      final velocity = previousVelocities[ball.id]!;
      final velocityResidual = velocity - axis * velocity.dot(axis);
      if (positionResidual.length > 1e-6 || velocityResidual.length > 1e-4) {
        collinear = false;
        break;
      }
      if (previousAngularVelocities[ball.id]!.length > 1e-4) {
        spinFree = false;
        break;
      }
    }
    if (!collinear || !spinFree) continue;

    final initialPositions = <double>[];
    final initialVelocities = <double>[];
    var hasClosingContact = false;
    var validSpacing = true;
    for (var i = 0; i < ordered.length; i++) {
      final ball = balls[ordered[i]];
      final position = previousPositions[ball.id]!;
      final velocity = previousVelocities[ball.id]!;
      initialPositions.add(position.dot(axis));
      initialVelocities.add(velocity.dot(axis));
      if (i == 0) continue;
      final gap = initialPositions[i] - initialPositions[i - 1];
      final left = balls[ordered[i - 1]];
      final right = balls[ordered[i]];
      final initiallyTouching = gap <= 2 * _R + 1e-6;
      final hasImpactThisStep = _ballBallTimeOfImpactFraction(
            previousPositions[left.id]!,
            previousPositions[right.id]!,
            left.pos3,
            right.pos3,
          ) !=
          null;
      if (gap < 2 * _R - 0.002 || (!initiallyTouching && !hasImpactThisStep)) {
        validSpacing = false;
        break;
      }
      if (initialVelocities[i - 1] - initialVelocities[i] > 1e-6) {
        hasClosingContact = true;
      }
    }
    if (!validSpacing || !hasClosingContact) continue;

    final result = integrateHertzContactChain(
      positions: initialPositions,
      velocities: initialVelocities,
      durationSeconds: durationSeconds,
      normalRestitution: _eBall,
    );
    Vec2? clusterCueContact;
    final cueIndex = ordered.indexWhere((i) => balls[i].id == cueBallId);
    if (cueIndex >= 0) {
      double? firstImpactFraction;
      for (final neighborIndex in [cueIndex - 1, cueIndex + 1]) {
        if (neighborIndex < 0 || neighborIndex >= ordered.length) continue;
        final cueBall = balls[ordered[cueIndex]];
        final neighbor = balls[ordered[neighborIndex]];
        final cueStart = previousPositions[cueBall.id]!;
        final neighborStart = previousPositions[neighbor.id]!;
        final fraction = (neighborStart - cueStart).length <= 2 * _R + 1e-6
            ? 0.0
            : _ballBallTimeOfImpactFraction(
                cueStart,
                neighborStart,
                cueBall.pos3,
                neighbor.pos3,
              );
        if (fraction != null &&
            (firstImpactFraction == null || fraction < firstImpactFraction)) {
          firstImpactFraction = fraction;
        }
      }
      if (firstImpactFraction != null) {
        final cueBall = balls[ordered[cueIndex]];
        final start = previousPositions[cueBall.id]!;
        clusterCueContact = Vec2(
          start.x + (cueBall.pos3.x - start.x) * firstImpactFraction,
          start.y + (cueBall.pos3.y - start.y) * firstImpactFraction,
        );
      }
    }
    final perpendicularOffset = origin - axis * origin.dot(axis);
    for (var i = 0; i < ordered.length; i++) {
      final ball = balls[ordered[i]];
      ball.pos3 = perpendicularOffset + axis * result.positions[i];
      ball.vel3 = axis * result.velocities[i];
      ball.angularVelocity = previousAngularVelocities[ball.id]!;
      handledIds.add(ball.id);
    }
    cueContact ??= clusterCueContact;
  }

  return _HertzChainResolution(handledIds, cueContact);
}

Vec2? _resolveCollisions(
  List<_Ball> balls,
  String cueBallId,
  _Cfg cfg, {
  Map<String, Vec3>? previousPositions,
  Map<String, Vec3>? previousVelocities,
  Map<String, Vec3>? previousAngularVelocities,
  double ballBallFrictionScale = 1.0,
  double timeSec = 0.0,
  double timeStepSec = _dt,
  List<BallBallContact>? contactDetails,
}) {
  final hertz = _resolveHertzChains(
    balls,
    cueBallId,
    previousPositions,
    previousVelocities,
    previousAngularVelocities,
    timeStepSec,
  );
  Vec2? firstContact = hertz.cueContact;
  for (int i = 0; i < balls.length; i++) {
    for (int j = i + 1; j < balls.length; j++) {
      final a = balls[i];
      final b = balls[j];
      if (!a.active || !b.active) continue;
      if (hertz.ballIds.contains(a.id) && hertz.ballIds.contains(b.id)) {
        continue;
      }

      final previousA = previousPositions?[a.id];
      final previousB = previousPositions?[b.id];
      final currentA = a.pos3;
      final currentB = b.pos3;
      final timeOfImpact = previousA != null && previousB != null
          ? _ballBallTimeOfImpactFraction(
              previousA,
              previousB,
              currentA,
              currentB,
            )
          : null;
      final impactA = timeOfImpact == null
          ? currentA
          : previousA! + (currentA - previousA) * timeOfImpact;
      final impactB = timeOfImpact == null
          ? currentB
          : previousB! + (currentB - previousB) * timeOfImpact;
      final delta3 = impactB - impactA;
      final dist = delta3.length;
      if ((dist < 2 * _R || timeOfImpact != null) && dist > 1e-6) {
        final n3 = delta3 / dist;
        final horizontalLength = math.sqrt(n3.x * n3.x + n3.y * n3.y);
        final n = horizontalLength > 1e-9
            ? Vec2(n3.x / horizontalLength, n3.y / horizontalLength)
            : Vec2.zero;
        final vRel = a.vel3 - b.vel3;
        final vN = vRel.dot(n3);
        if (vN <= 0) continue;
        final velocityBeforeImpactA = a.vel3;
        final velocityBeforeImpactB = b.vel3;
        final angularVelocityBeforeImpactA = a.angularVelocity;
        final angularVelocityBeforeImpactB = b.angularVelocity;
        final isInitialFrozenCuePair = timeSec <= timeStepSec + 1e-9 &&
            (a.id == cueBallId || b.id == cueBallId) &&
            previousPositions != null &&
            (previousPositions[b.id]! - previousPositions[a.id]!).length <=
                2 * _R + 1e-8;

        final overlap = 2 * _R - dist;
        if (timeOfImpact != null) {
          a.pos3 = impactA;
          b.pos3 = impactB;
        } else if (overlap > 0) {
          a.pos3 -= n3 * (overlap / 2);
          b.pos3 += n3 * (overlap / 2);
        }

        // A cue ball struck while already frozen to an object ball is a
        // composite two-ball system in the normal direction: the cue impulse
        // is shared, so both balls initially take half the normal speed. This
        // differs from an isolated ball-ball impact (e=0.93), where the cue
        // ball nearly stops and the object ball takes nearly all of it.
        final normalRestitution = isInitialFrozenCuePair ? 0.0 : _eBall;
        final jN = (1 + normalRestitution) * 0.5 * vN;
        a.vel3 -= n3 * jN;
        b.vel3 += n3 * jN;

        if (a.id == cueBallId || b.id == cueBallId) {
          firstContact ??= (a.id == cueBallId ? a : b).pos;
        }

        final rA = n3 * _R;
        final rB = -rA;
        final contactA = a.vel3 + a.angularVelocity.cross(rA);
        final contactB = b.vel3 + b.angularVelocity.cross(rB);
        final contactRelative = contactA - contactB;
        final contactTangent = contactRelative - n3 * contactRelative.dot(n3);
        final frictionCoefficient =
            ballBallFrictionForSlip(contactTangent.length) *
                ballBallFrictionScale;
        var tangentialImpulseMagnitude = 0.0;
        var tangentialImpulseOnA = Vec3.zero;

        if (contactTangent.length > 1e-9) {
          final effectiveInvMass = 2.0 + 2.0 / _ballInertia;
          final jTstop = -contactTangent / effectiveInvMass;
          final jTmax = frictionCoefficient * jN;
          final impulse =
              jTstop.length <= jTmax ? jTstop : jTstop.normalized() * jTmax;
          tangentialImpulseOnA = impulse;
          tangentialImpulseMagnitude = impulse.length;
          a.vel3 += impulse;
          b.vel3 -= impulse;
          a.angularVelocity += rA.cross(impulse) / (_ballInertia * _R * _R);
          b.angularVelocity += rB.cross(-impulse) / (_ballInertia * _R * _R);
        }

        if (contactDetails != null &&
            (a.id == cueBallId || b.id == cueBallId) &&
            horizontalLength > 1e-9) {
          final cueIsA = a.id == cueBallId;
          final cueBall = cueIsA ? a : b;
          final objectBall = cueIsA ? b : a;
          final impactNormal = cueIsA ? n : -n;
          final cueSlip = cueIsA ? contactTangent : -contactTangent;
          final cueTangentialImpulse =
              cueIsA ? tangentialImpulseOnA : -tangentialImpulseOnA;
          final tangent = Vec2(-impactNormal.y, impactNormal.x);
          final objectVelocity = objectBall.vel;
          contactDetails.add(
            BallBallContact(
              timeSec: timeOfImpact == null
                  ? timeSec
                  : timeSec - timeStepSec * (1 - timeOfImpact),
              cueBallId: cueBallId,
              objectBallId: objectBall.id,
              impactNormal: impactNormal,
              impactNormal3: cueIsA ? n3 : -n3,
              incomingNormalSpeedMs: vN,
              relativeTangentialSlipSpeedMs: cueSlip.length,
              relativeTangentialSlip: cueSlip,
              frictionCoefficient: frictionCoefficient,
              normalImpulsePerMass: jN,
              tangentialImpulsePerMass: tangentialImpulseMagnitude,
              tangentialImpulseOnCueBall: cueTangentialImpulse,
              throwAngleRad: math.atan2(
                objectVelocity.dot(tangent),
                objectVelocity.dot(impactNormal),
              ),
              cuePreImpactVelocity:
                  cueIsA ? velocityBeforeImpactA : velocityBeforeImpactB,
              cuePreImpactAngularVelocity: cueIsA
                  ? angularVelocityBeforeImpactA
                  : angularVelocityBeforeImpactB,
              cuePostImpactVelocity: cueBall.vel3,
              cuePostImpactAngularVelocity: cueBall.angularVelocity,
              objectPostImpactAngularVelocity: objectBall.angularVelocity,
            ),
          );
        }

        if (timeOfImpact != null) {
          final remainingTime = timeStepSec * (1 - timeOfImpact);
          a.pos3 += a.vel3 * remainingTime;
          b.pos3 += b.vel3 * remainingTime;
        }
      }
    }
  }
  return firstContact;
}

double? _ballBallTimeOfImpactFraction(Vec3 a0, Vec3 b0, Vec3 a1, Vec3 b1) {
  final separation0 = b0 - a0;
  final separationDelta = (b1 - a1) - separation0;
  final aa = separationDelta.lengthSquared;
  final bb = 2 * separation0.dot(separationDelta);
  final cc = separation0.lengthSquared - 4 * _R * _R;
  if (aa < 1e-18 || cc < -1e-12 || bb >= 0) return null;

  final discriminant = bb * bb - 4 * aa * cc;
  if (discriminant < 0) return null;
  final fraction = (-bb - math.sqrt(discriminant)) / (2 * aa);
  if (fraction < -1e-9 || fraction > 1 + 1e-9) return null;
  return fraction.clamp(0.0, 1.0);
}

// ── Cushion helper ────────────────────────────────────────────────────────────

void _limitAngularSpeed(_Ball b) {
  final maxRadPerSec = maxPhysicalSpinRpm / _radiansPerSecondToRpm;
  final speed = b.angularVelocity.length;
  if (speed > maxRadPerSec) {
    b.angularVelocity = b.angularVelocity * (maxRadPerSec / speed);
  }
}

void _applyCushionImpact(
  _Ball b,
  Vec3 normal,
  Vec3 tangent,
  CushionProfile profile,
) {
  // Mathavan, Jackson & Parkin's 3-D impulse model applies while the ball is
  // supported by the cloth; a separate impact path handles airborne contacts.
  if (b.pos3.z <= _R + 1e-6 && b.vel3.z.abs() <= 1e-5) {
    _applyGroundedCushionImpact(b, normal, profile);
    return;
  }

  _applyLegacyCushionImpact(
    b,
    normal,
    tangent,
    profile,
    profile.airborneRestitution,
    includeVerticalFriction: b.pos3.z > _R + 0.001,
  );
}

void _applyGroundedCushionImpact(
  _Ball b,
  Vec3 inwardNormal,
  CushionProfile profile,
) {
  const mass = _cueBallMassKg;
  const slipTolerance = 4e-4;
  final sinTheta =
      ((profile.noseHeightMm / 1000.0 - _R) / _R).clamp(0.05, 0.95).toDouble();
  final tableImpactFriction = profile.tableImpactFriction;
  final cosTheta = math.sqrt(1.0 - sinTheta * sinTheta);
  final up = const Vec3(0, 0, 1);
  final outward = inwardNormal * -1.0;
  // The UI/table plane stores y downward (left-handed with z up). Convert the
  // tangent basis and axial spin components to the paper's right-handed frame.
  final ex = up.cross(outward);
  final ey = outward;
  var vx = b.vel3.dot(ex);
  var vy = b.vel3.dot(ey);
  var wx = -b.angularVelocity.dot(ex);
  var wy = -b.angularVelocity.dot(ey);
  var wz = -b.angularVelocity.z;

  final approach = vy;
  if (approach <= 0) return;

  // P_I is the independent impulse in Mathavan's equations. Its horizontal
  // normal component is cos(theta), hence this estimate for total impulse.
  final estimatedImpulse =
      (1.0 + profile.restitution) * mass * approach / cosTheta;
  final impulseStep = estimatedImpulse / 2048.0;
  var accumulatedWork = 0.0;
  var compressionWork = 0.0;
  var targetFinalWork = 0.0;
  var compressionEnded = false;

  for (int i = 0; i < 12000; i++) {
    // Slip at cushion contact I and cloth contact C, equations 12–13.
    final slipRailX = vx + _R * (wy * sinTheta - wz * cosTheta);
    final slipRailY = -vy * sinTheta + wx * _R;
    final slipRail = math.sqrt(slipRailX * slipRailX + slipRailY * slipRailY);
    final hasRailSlip = slipRail > slipTolerance;
    final phi = hasRailSlip ? math.atan2(slipRailY, slipRailX) : 0.0;

    final slipTableX = vx - wy * _R;
    final slipTableY = vy + wx * _R;
    final slipTable = math.sqrt(
      slipTableX * slipTableX + slipTableY * slipTableY,
    );
    final hasTableSlip = slipTable > slipTolerance;
    final phiTable = hasTableSlip ? math.atan2(slipTableY, slipTableX) : 0.0;

    final muRailX = hasRailSlip ? profile.slidingFriction * math.cos(phi) : 0.0;
    final muRailY = hasRailSlip ? profile.slidingFriction * math.sin(phi) : 0.0;
    final tableNormalFactor = sinTheta + muRailY * cosTheta;
    final muTableX =
        hasTableSlip ? tableImpactFriction * math.cos(phiTable) : 0.0;
    final muTableY =
        hasTableSlip ? tableImpactFriction * math.sin(phiTable) : 0.0;

    // Restitution work is measured along the common contact normal, not the
    // tangential slip velocity at either contact.
    final oldNormalSpeed = -vy * cosTheta;
    final dvx = -(muRailX + muTableX * tableNormalFactor) / mass;
    final dvy = -(cosTheta -
            profile.slidingFriction * sinTheta * math.sin(phi) +
            muTableY * tableNormalFactor) /
        mass;
    final dwx = -2.5 / (mass * _R) * (muRailY + muTableY * tableNormalFactor);
    final dwy = -2.5 /
        (mass * _R) *
        (muRailX * sinTheta - muTableX * tableNormalFactor);
    final dwz = 2.5 / (mass * _R) * muRailX * cosTheta;

    vx += dvx * impulseStep;
    vy += dvy * impulseStep;
    wx += dwx * impulseStep;
    wy += dwy * impulseStep;
    wz += dwz * impulseStep;

    final newNormalSpeed = -vy * cosTheta;
    accumulatedWork += impulseStep * (oldNormalSpeed + newNormalSpeed) * 0.5;

    // Stronge's energetic COR: ee² = -W_restitution / W_compression.
    // Compression work is negative; stop restitution when total work reaches
    // (1-ee²) times that compression work.
    if (!compressionEnded && newNormalSpeed >= 0) {
      compressionEnded = true;
      compressionWork = accumulatedWork;
      targetFinalWork =
          (1.0 - profile.restitution * profile.restitution) * compressionWork;
    } else if (compressionEnded && accumulatedWork >= targetFinalWork) {
      break;
    }
  }

  b.vel3 = ex * vx + ey * vy;
  b.angularVelocity = (ex * wx + ey * wy + up * wz) * -1.0;
}

void _applyLegacyCushionImpact(
  _Ball b,
  Vec3 normal,
  Vec3 tangent,
  CushionProfile profile,
  double restitution, {
  bool includeVerticalFriction = true,
}) {
  final approach = -b.vel3.dot(normal);
  if (approach <= 0) return;

  final normalImpulse = (1 + restitution) * approach;
  final samples = math.max(8, (profile.impactDurationMs / 0.5).round());
  final noseOffset = profile.noseHeightMm / 1000.0 - _R;
  final r = normal * -_R + Vec3(0, 0, noseOffset);
  final inertia = _ballInertia * _R * _R;
  const verticalTangent = Vec3(0, 0, 1);

  Vec3 contactVelocityChangePerImpulse(Vec3 impulsePerMass) =>
      impulsePerMass + r.cross(impulsePerMass).cross(r) / inertia;

  final responseAlongRail = contactVelocityChangePerImpulse(tangent);
  final responseVertical = contactVelocityChangePerImpulse(verticalTangent);
  final k11 = responseAlongRail.dot(tangent);
  final k12 = responseVertical.dot(tangent);
  final k21 = responseAlongRail.dot(verticalTangent);
  final k22 = responseVertical.dot(verticalTangent);
  final determinant = k11 * k22 - k12 * k21;

  for (int i = 0; i < samples; i++) {
    final phase = (i + 0.5) / samples;
    final jN =
        normalImpulse * math.sin(math.pi * phase) * math.pi / (2 * samples);
    final mu = _cushionGrip(profile, approach);
    final contactVelocity = b.vel3 + b.angularVelocity.cross(r);
    final slipAlongRail = contactVelocity.dot(tangent);
    final Vec3 stopImpulse;
    final double? legacyStopImpulse;
    if (includeVerticalFriction) {
      legacyStopImpulse = null;
      final slipVertical = contactVelocity.dot(verticalTangent);
      final desiredAlongRail = -slipAlongRail;
      final desiredVertical = -slipVertical;
      final stopAlongRail =
          (desiredAlongRail * k22 - k12 * desiredVertical) / determinant;
      final stopVertical =
          (k11 * desiredVertical - k21 * desiredAlongRail) / determinant;
      stopImpulse = tangent * stopAlongRail + verticalTangent * stopVertical;
    } else {
      final rotationalLever = r.cross(tangent).lengthSquared / inertia;
      legacyStopImpulse = -slipAlongRail / (1.0 + rotationalLever);
      stopImpulse = tangent * legacyStopImpulse;
    }
    final frictionLimit = mu * jN;
    final Vec3 tangentialImpulse;
    if (includeVerticalFriction) {
      tangentialImpulse = stopImpulse.length <= frictionLimit
          ? stopImpulse
          : stopImpulse.normalized() * frictionLimit;
    } else {
      // Preserve the original scalar rail-axis impulse for pocket facings;
      // their tangent vectors need not be unit length.
      final jT = legacyStopImpulse!;
      final limitedJT =
          jT.abs() <= frictionLimit ? jT : -frictionLimit * jT.sign;
      tangentialImpulse = tangent * limitedJT;
    }
    final impulse = normal * jN + tangentialImpulse;
    b.vel3 += impulse;
    final angularImpulse = r.cross(impulse) / inertia;
    final spinRatio = (b.spinZ.abs() * _R) / math.max(b.vel3.length, 1e-6);
    final spinTransfer = profile.tangentialSpinTransfer +
        (1.0 - profile.tangentialSpinTransfer) *
            _smoothStep((spinRatio - 0.35) / 0.65);
    b.angularVelocity += Vec3(
      angularImpulse.x,
      angularImpulse.y,
      angularImpulse.z * spinTransfer,
    );
  }

  final tangentVelocity = b.vel3.dot(tangent);
  final speedRetention = _cushionTangentialRetention(
    profile,
    b.vel3.length,
    b.spinZ,
  );
  b.vel3 -= tangent * (tangentVelocity * (1.0 - speedRetention));
  b.spinZ *= speedRetention;
}

double _cushionTangentialRetention(
  CushionProfile profile,
  double incidentSpeed,
  double spinZ,
) {
  final t = _smoothStep((incidentSpeed - 2.60) / (13.4112 - 2.60));
  final base = profile.tangentialRestitution +
      (profile.highSpeedTangentialRestitution - profile.tangentialRestitution) *
          t;
  final spinRatio = (spinZ.abs() * _R) / math.max(incidentSpeed, 1e-6);
  final spinRetention = 0.60 * _smoothStep((spinRatio - 0.35) / 0.65);
  return (base + spinRetention).clamp(0.0, 0.95);
}

double _smoothStep(double x) {
  final t = x.clamp(0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

double _cushionGrip(CushionProfile profile, double approach) {
  if (approach <= profile.peakGripSpeed) {
    return profile.slidingFriction;
  }
  final t = _smoothStep(
    (approach - profile.peakGripSpeed) /
        (profile.highSpeedReference - profile.peakGripSpeed),
  );
  return profile.slidingFriction +
      (profile.highSpeedFriction - profile.slidingFriction) * t;
}

int _resolveCushions(
  List<_Ball> balls,
  _Cfg cfg,
  CushionProfile profile, {
  Set<String>? contactIds,
  List<CushionContact>? contactDetails,
  double timeSec = 0.0,
}) {
  var contacts = 0;

  void impact(_Ball b, Vec3 normal, Vec3 tangent, String rail) {
    final approach = -b.vel3.dot(normal);
    if (approach <= 0) return;
    final tangentBefore = b.vel3.dot(tangent);
    final incidentSpeed = b.vel3.length;
    final incidentAngle =
        math.atan2(tangentBefore.abs(), approach) * 180.0 / math.pi;
    final spinBefore = b.angularVelocity.length * _radiansPerSecondToRpm;
    final spinZBefore = b.spinZ;

    _applyCushionImpact(b, normal, tangent, profile);
    _limitAngularSpeed(b);

    final reboundNormal = b.vel3.dot(normal).abs();
    final tangentAfter = b.vel3.dot(tangent);
    final reboundAngle =
        math.atan2(tangentAfter.abs(), reboundNormal) * 180.0 / math.pi;
    contactDetails?.add(
      CushionContact(
        timeSec: timeSec,
        ballId: b.id,
        rail: rail,
        positionRel: _metersToRel(b.pos, cfg),
        incidentAngleDeg: incidentAngle,
        reboundAngleDeg: reboundAngle,
        incidentSpeedMs: incidentSpeed,
        reboundSpeedMs: b.vel3.length,
        spinRpmBefore: spinBefore,
        spinRpmAfter: b.angularVelocity.length * _radiansPerSecondToRpm,
        spinZBefore: spinZBefore,
        spinZAfter: b.spinZ,
      ),
    );
  }

  for (final b in balls) {
    if (!b.active) continue;

    if (b.pos.x < _R && _railFacePresentAtEdge('left', b.pos, cfg)) {
      b.pos = Vec2(_R, b.pos.y);
      if (-b.vel3.x > 0) {
        contacts++;
        contactIds?.add(b.id);
        impact(b, const Vec3(1, 0, 0), const Vec3(0, 1, 0), 'left');
      }
    }

    if (b.pos.x > cfg.W - _R && _railFacePresentAtEdge('right', b.pos, cfg)) {
      b.pos = Vec2(cfg.W - _R, b.pos.y);
      if (b.vel3.x > 0) {
        contacts++;
        contactIds?.add(b.id);
        impact(b, const Vec3(-1, 0, 0), const Vec3(0, 1, 0), 'right');
      }
    }

    if (b.pos.y < _R && _railFacePresentAtEdge('top', b.pos, cfg)) {
      b.pos = Vec2(b.pos.x, _R);
      if (-b.vel3.y > 0) {
        contacts++;
        contactIds?.add(b.id);
        impact(b, const Vec3(0, 1, 0), const Vec3(1, 0, 0), 'top');
      }
    }

    if (b.pos.y > cfg.H - _R && _railFacePresentAtEdge('bottom', b.pos, cfg)) {
      b.pos = Vec2(b.pos.x, cfg.H - _R);
      if (b.vel3.y > 0) {
        contacts++;
        contactIds?.add(b.id);
        impact(b, const Vec3(0, -1, 0), const Vec3(1, 0, 0), 'bottom');
      }
    }
  }
  return contacts;
}

// Decide whether the straight cushion face exists at this contact projection.
// A radial exclusion around a pocket wrongly removes valid rail contact beside
// the jaw: a ball tangent to the straight rail can then be caught by the jaw's
// rounded endpoint instead. The nose ends cornerJawR from each corner;
// side-pocket openings are handled separately on the top/bottom rails.
bool _railFacePresentAtEdge(String edge, Vec2 pos, _Cfg cfg) {
  final cornerEnd = cfg.cornerJawR;
  switch (edge) {
    case 'left':
    case 'right':
      return pos.y >= cornerEnd && pos.y <= cfg.H - cornerEnd;
    case 'top':
    case 'bottom':
      return pos.x >= cornerEnd &&
          pos.x <= cfg.W - cornerEnd &&
          !_nearSidePocket(pos, cfg);
    default:
      return false;
  }
}

List<_CornerFrame> _cornerFrames(_Cfg cfg) => [
      const _CornerFrame('topLeft', Vec2.zero, 1, 1),
      _CornerFrame('topRight', Vec2(cfg.W, 0), -1, 1),
      _CornerFrame('bottomLeft', Vec2(0, cfg.H), 1, -1),
      _CornerFrame('bottomRight', Vec2(cfg.W, cfg.H), -1, -1),
    ];

List<_PocketSegment> _cornerFacingSegments(_Cfg cfg) {
  final leg = cfg.cornerJawR;
  final length = cfg.cornerFacingLength;
  final dx = math.cos(cfg.cornerFacingAngle) * length;
  final dy = math.sin(cfg.cornerFacingAngle) * length;
  final result = <_PocketSegment>[];
  for (final corner in _cornerFrames(cfg)) {
    result.add(
      _PocketSegment(
        corner,
        'longRailFacing',
        corner.toWorld(Vec2(leg, 0)),
        corner.toWorld(Vec2(leg - dx, -dy)),
      ),
    );
    result.add(
      _PocketSegment(
        corner,
        'shortRailFacing',
        corner.toWorld(Vec2(0, leg)),
        corner.toWorld(Vec2(-dy, leg - dx)),
      ),
    );
  }
  return result;
}

// The ball's centre drops as soon as it enters the rendered black hole. The
// facings can still reject it before that boundary; once its centre crosses
// the circle, the ball is no longer supported by the shelf.
_CornerFrame? _capturingCorner(Vec2 pos, _Cfg cfg) {
  final captureRadius = cfg.cornerHoleR;
  for (final corner in _cornerFrames(cfg)) {
    final holeCenter = corner.toWorld(
      Vec2(-cfg.cornerHoleOffset, -cfg.cornerHoleOffset),
    );
    if ((pos - holeCenter).distance < captureRadius) return corner;
  }
  return null;
}

bool _onCornerShelfOrBetweenJaws(Vec2 pos, _Cfg cfg) {
  if (_capturingCorner(pos, cfg) != null) return false;
  final leg = cfg.cornerJawR;
  final outsideLimit = cfg.cornerHoleR;
  for (final corner in _cornerFrames(cfg)) {
    final local = corner.toLocal(pos);
    if (local.x + local.y < leg &&
        local.x >= -outsideLimit &&
        local.y >= -outsideLimit &&
        local.x <= leg &&
        local.y <= leg) {
      return true;
    }
  }
  return false;
}

bool _nearSidePocket(Vec2 pos, _Cfg cfg) {
  final hw = cfg.sidePocketHalfW;
  final d = cfg.sidePocketDepth;
  return ((pos.x - cfg.W / 2).abs() < hw && pos.y < d) ||
      ((pos.x - cfg.W / 2).abs() < hw && pos.y > cfg.H - d);
}

// True when the ball's physical edge has cleared the jaw rubber (i.e. it is
// fully inside the pocket mouth and cannot be rejected by the jaw point).
// Uses hw - _R so that a ball whose center is within _R of the jaw is NOT
// considered inside the mouth — the jaw handles it via _resolveJawPoints.
bool _inSidePocketMouth(Vec2 pos, _Cfg cfg) {
  final hw = cfg.sidePocketHalfW - _R;
  final d = cfg.sidePocketDepth;
  return ((pos.x - cfg.W / 2).abs() < hw && pos.y < d) ||
      ((pos.x - cfg.W / 2).abs() < hw && pos.y > cfg.H - d);
}

// The four rubber nose-tips at the edges of the two side pocket openings.
List<Vec2> _sideJawPoints(_Cfg cfg) {
  final hw = cfg.sidePocketHalfW;
  return [
    Vec2(cfg.W / 2 - hw, 0),
    Vec2(cfg.W / 2 + hw, 0),
    Vec2(cfg.W / 2 - hw, cfg.H),
    Vec2(cfg.W / 2 + hw, cfg.H),
  ];
}

Vec2 _closestPointOnSegment(Vec2 point, Vec2 start, Vec2 end) {
  final segment = end - start;
  final lengthSquared = segment.x * segment.x + segment.y * segment.y;
  if (lengthSquared < 1e-16) return start;
  final offset = point - start;
  final t = ((offset.x * segment.x + offset.y * segment.y) / lengthSquared)
      .clamp(0.0, 1.0);
  return start + segment * t;
}

// Resolve the rounded side-pocket jaw tips and the full corner-facing segments.
// A segment naturally includes its jaw endpoint, replacing the old point-only
// corner model while retaining a rounded collision at the cushion tip.
int _resolvePocketGeometry(
  List<_Ball> balls,
  _Cfg cfg,
  CushionProfile profile, {
  Set<String>? contactIds,
  List<PocketContact>? contactDetails,
  double timeSec = 0.0,
}) {
  var contacts = 0;
  final surfacesThisStep = <String, Set<String>>{};

  void impact(
    _Ball ball,
    Vec2 closest,
    String pocket,
    String surface,
    Vec2? segmentDirection,
  ) {
    final delta = ball.pos - closest;
    final distance = delta.distance;
    if (distance >= _R || distance < 1e-8) return;
    final normal2 = delta / distance;
    final normal3 = Vec3(normal2.x, normal2.y, 0);
    if (-ball.vel3.dot(normal3) <= 0) return;

    final surfaceKey = '$pocket/$surface';
    surfacesThisStep.putIfAbsent(ball.id, () => <String>{}).add(surfaceKey);
    final isNewContact = !ball.activePocketSurfaces.contains(surfaceKey);

    final incidentSpeed = ball.vel3.length;
    ball.pos = closest + normal2 * _R;
    final tangent2 = segmentDirection ?? Vec2(-normal2.y, normal2.x);
    _applyLegacyCushionImpact(
      ball,
      normal3,
      Vec3(tangent2.x, tangent2.y, 0),
      profile,
      profile.pocketFacingRestitution,
      includeVerticalFriction: false,
    );
    _limitAngularSpeed(ball);
    contacts++;
    contactIds?.add(ball.id);
    if (isNewContact) {
      contactDetails?.add(
        PocketContact(
          timeSec: timeSec,
          ballId: ball.id,
          pocket: pocket,
          surface: surface,
          positionRel: _metersToRel(ball.pos, cfg),
          incidentSpeedMs: incidentSpeed,
          reboundSpeedMs: ball.vel3.length,
        ),
      );
    }
  }

  final facings = _cornerFacingSegments(cfg);
  for (final b in balls) {
    if (!b.active) continue;
    for (final facing in facings) {
      final closest = _closestPointOnSegment(b.pos, facing.start, facing.end);
      final direction = (facing.end - facing.start).normalized;
      impact(b, closest, facing.corner.name, facing.surface, direction);
    }
    for (final jaw in _sideJawPoints(cfg)) {
      final top = jaw.y < cfg.H / 2;
      impact(b, jaw, top ? 'topSide' : 'bottomSide', 'jawTip', null);
    }
  }
  for (final b in balls) {
    b.activePocketSurfaces = surfacesThisStep[b.id] ?? <String>{};
  }

  return contacts;
}

void _checkPockets(List<_Ball> balls, _Cfg cfg) {
  for (final b in balls) {
    if (!b.active) continue;
    if (b.pos3.z > _R + 0.002) continue;
    // Corner pockets: ball must reach the throat (inner zone past the jaws).
    // Side pockets: ball must clear the jaw mouth (stricter zone).
    if (_capturingCorner(b.pos, cfg) != null ||
        _inSidePocketMouth(b.pos, cfg)) {
      b.active = false;
      b.pocketed = true;
      b.pocketPosition = _pocketCenter(b.pos, cfg);
    }
  }
}

Vec2 _pocketCenter(Vec2 pos, _Cfg cfg) {
  final cornerX = pos.x < cfg.W / 2 ? 0.0 : cfg.W;
  final cornerY = pos.y < cfg.H / 2 ? 0.0 : cfg.H;
  final corner = Vec2(cornerX, cornerY);
  if ((pos - corner).distance < cfg.cornerPocketDetectR + cfg.cornerHoleR) {
    return corner;
  }
  return Vec2(cfg.W / 2, pos.y < cfg.H / 2 ? 0.0 : cfg.H);
}
