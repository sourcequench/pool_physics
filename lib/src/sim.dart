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

// ── Table config ─────────────────────────────────────────────────────────────

class _Cfg {
  final double W;
  final double H;
  final double minimumCueElevationDeg;
  final double clothSlidingFriction;
  final double clothRollingResistance;
  final double radius;
  final bool hasPockets;
  final double relX0;
  final double relX1;
  final double relY0;
  final double relY1;

  final double cornerJawR;
  final double cornerHoleR;
  final double cornerShelf;
  final double cornerFacingAngle;
  final double cornerHoleOffset;
  final double cornerFacingLength;
  final double cornerPocketDetectR;
  final double sidePocketHalfW;
  final double sideHoleR;
  final double sideShelf;
  final double sideFacingAngle;
  final double sideHoleOffset;
  final double sideFacingLength;
  final double sidePocketDepth;
  final double pocketCheckMargin;
  final double rollingTurnFactor;
  final List<_CornerFrame> cornerFrames;
  final List<_PocketSegment> pocketFacings;

  _Cfg._({
    required this.W,
    required this.H,
    required this.minimumCueElevationDeg,
    required this.clothSlidingFriction,
    required this.clothRollingResistance,
    required this.radius,
    required this.hasPockets,
    required this.relX0,
    required this.relX1,
    required this.relY0,
    required this.relY1,
    required this.cornerJawR,
    required this.cornerHoleR,
    required this.cornerShelf,
    required this.cornerFacingAngle,
    required this.cornerHoleOffset,
    required this.cornerFacingLength,
    required this.cornerPocketDetectR,
    required this.sidePocketHalfW,
    required this.sideHoleR,
    required this.sideShelf,
    required this.sideFacingAngle,
    required this.sideHoleOffset,
    required this.sideFacingLength,
    required this.sidePocketDepth,
    required this.pocketCheckMargin,
    required this.rollingTurnFactor,
    required this.cornerFrames,
    required this.pocketFacings,
  });

  static _Cfg from(PhysicsTableConfig config) {
    final w = config.playW;
    final h = config.playH;
    final r = config.ballRadiusM;
    final cJawR = cornerMouthInches / math.sqrt2 * inchesToMeters;
    final cHoleR = cornerHoleRadiusInches * inchesToMeters;
    final cShelf = cornerShelfInches * inchesToMeters;
    final cFacingAngle = cornerFacingAngleDegrees * math.pi / 180.0;
    final cHoleOffset =
        ((cornerShelfInches + cornerHoleRadiusInches - cornerMouthInches / 2) /
                math.sqrt2) *
            inchesToMeters;
    final cosCA = math.cos(cFacingAngle);
    final sinCA = math.sin(cFacingAngle);
    final proj = (cJawR + cHoleOffset) * cosCA + cHoleOffset * sinCA;
    final distSq = (cJawR + cHoleOffset) * (cJawR + cHoleOffset) +
        cHoleOffset * cHoleOffset;
    final cFacingLen =
        proj - math.sqrt(proj * proj - (distSq - cHoleR * cHoleR));
    final cDetectR = cJawR + r * 1.2;

    final sHalfW = sideMouthInches * inchesToMeters / 2;
    final sHoleR = sideHoleRadiusInches * inchesToMeters;
    final sShelf = sideShelfInches * inchesToMeters;
    final sFacingAngle = sideFacingAngleDegrees * math.pi / 180.0;
    final sHoleOffset =
        (sideShelfInches + sideHoleRadiusInches) * inchesToMeters;
    final sFacingLen =
        pocketCushionDepthInches / math.cos(sFacingAngle) * inchesToMeters;
    final sDepth = 0.030 * w / 2.54;
    final margin = math.max(cJawR + cHoleR, sDepth + sHoleR) + r * 1.5;

    final muR = config.clothRollingResistance;
    final turnNormalAngle = math.atan(muR) +
        math.asin((2.0 / 5.0) * muR / math.sqrt(1.0 + muR * muR));
    final turnFactor =
        math.sin(turnNormalAngle) / (2.0 / 5.0 + math.cos(turnNormalAngle));

    final frames = <_CornerFrame>[
      _CornerFrame(
        'topLeft',
        Vec2.zero,
        1,
        1,
        Vec2(-cHoleOffset, -cHoleOffset),
      ),
      _CornerFrame(
        'topRight',
        Vec2(w, 0),
        -1,
        1,
        Vec2(w + cHoleOffset, -cHoleOffset),
      ),
      _CornerFrame(
        'bottomLeft',
        Vec2(0, h),
        1,
        -1,
        Vec2(-cHoleOffset, h + cHoleOffset),
      ),
      _CornerFrame(
        'bottomRight',
        Vec2(w, h),
        -1,
        -1,
        Vec2(w + cHoleOffset, h + cHoleOffset),
      ),
    ];

    final facings = <_PocketSegment>[];
    final cdx = cosCA * cFacingLen;
    final cdy = sinCA * cFacingLen;
    for (final corner in frames) {
      facings.add(
        _PocketSegment(
          corner.name,
          'longRailFacing',
          corner.toWorld(Vec2(cJawR, 0)),
          corner.toWorld(Vec2(cJawR - cdx, -cdy)),
        ),
      );
      facings.add(
        _PocketSegment(
          corner.name,
          'shortRailFacing',
          corner.toWorld(Vec2(0, cJawR)),
          corner.toWorld(Vec2(-cdy, cJawR - cdx)),
        ),
      );
    }
    final sdx = math.sin(sFacingAngle) * sFacingLen;
    final sdy = math.cos(sFacingAngle) * sFacingLen;
    facings.addAll([
      _PocketSegment(
        'topSide',
        'leftFacing',
        Vec2(w / 2 - sHalfW, 0),
        Vec2(w / 2 - sHalfW + sdx, -sdy),
      ),
      _PocketSegment(
        'topSide',
        'rightFacing',
        Vec2(w / 2 + sHalfW, 0),
        Vec2(w / 2 + sHalfW - sdx, -sdy),
      ),
      _PocketSegment(
        'bottomSide',
        'leftFacing',
        Vec2(w / 2 - sHalfW, h),
        Vec2(w / 2 - sHalfW + sdx, h + sdy),
      ),
      _PocketSegment(
        'bottomSide',
        'rightFacing',
        Vec2(w / 2 + sHalfW, h),
        Vec2(w / 2 + sHalfW - sdx, h + sdy),
      ),
    ]);

    return _Cfg._(
      W: w,
      H: h,
      minimumCueElevationDeg: config.minimumCueElevationDeg,
      clothSlidingFriction: config.clothSlidingFriction,
      clothRollingResistance: muR,
      radius: r,
      hasPockets: config.hasPockets,
      relX0: config.relX0,
      relX1: config.relX1,
      relY0: config.relY0,
      relY1: config.relY1,
      cornerJawR: cJawR,
      cornerHoleR: cHoleR,
      cornerShelf: cShelf,
      cornerFacingAngle: cFacingAngle,
      cornerHoleOffset: cHoleOffset,
      cornerFacingLength: cFacingLen,
      cornerPocketDetectR: cDetectR,
      sidePocketHalfW: sHalfW,
      sideHoleR: sHoleR,
      sideShelf: sShelf,
      sideFacingAngle: sFacingAngle,
      sideHoleOffset: sHoleOffset,
      sideFacingLength: sFacingLen,
      sidePocketDepth: sDepth,
      pocketCheckMargin: margin,
      rollingTurnFactor: turnFactor,
      cornerFrames: frames,
      pocketFacings: facings,
    );
  }
}

class _CornerFrame {
  final String name;
  final Vec2 origin;
  final double xInward;
  final double yInward;
  final Vec2 holeCenter;

  const _CornerFrame(
    this.name,
    this.origin,
    this.xInward,
    this.yInward, [
    this.holeCenter = Vec2.zero,
  ]);

  Vec2 toWorld(Vec2 local) =>
      Vec2(origin.x + xInward * local.x, origin.y + yInward * local.y);

  Vec2 toLocal(Vec2 world) =>
      Vec2((world.x - origin.x) * xInward, (world.y - origin.y) * yInward);
}

class _PocketSegment {
  final String pocket;
  final String surface;
  final Vec2 start;
  final Vec2 end;
  final Vec2 direction;
  final double minX;
  final double maxX;
  final double minY;
  final double maxY;

  _PocketSegment(this.pocket, this.surface, this.start, this.end)
      : direction = (end - start).normalized,
        minX = math.min(start.x, end.x),
        maxX = math.max(start.x, end.x),
        minY = math.min(start.y, end.y),
        maxY = math.max(start.y, end.y);
}

// ── Coordinate helpers ───────────────────────────────────────────────────────

Vec2 _relToMeters(Vec2 rel, _Cfg cfg) => Vec2(
      (rel.x - cfg.relX0) / (cfg.relX1 - cfg.relX0) * cfg.W,
      (rel.y - cfg.relY0) / (cfg.relY1 - cfg.relY0) * cfg.H,
    );

Vec2 _metersToRel(Vec2 m, _Cfg cfg) => Vec2(
      m.x / cfg.W * (cfg.relX1 - cfg.relX0) + cfg.relX0,
      m.y / cfg.H * (cfg.relY1 - cfg.relY0) + cfg.relY0,
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
        (2 * cfg.radius) * (2 * cfg.radius);
    if (disc < 0) continue;
    final t = -dot - math.sqrt(disc);
    if (t > 0 && (minT == null || t < minT)) minT = t;
  }
  if (minT == null) return null;
  final ghostM = cueM + aimDirPhys * minT;
  if (ghostM.x < cfg.radius ||
      ghostM.x > cfg.W - cfg.radius ||
      ghostM.y < cfg.radius ||
      ghostM.y > cfg.H - cfg.radius) {
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
    // nose represented by the Mathavan profile. Diamond and tournament tables
    // use 3/16 in (4.8 mm) 60 Shore A neoprene facings over the wood sub-rail,
    // which has higher hysteresis (40–55% rebound resilience, effective COR
    // ~0.68–0.74) than natural gum cushion rubber (~40–45 Shore A, 65–80%+
    // resilience) so impacts deaden slightly rather than springing off the wood.
    this.pocketFacingRestitution = 0.72,
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
    pocketFacingRestitution: 0.70,
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

  // Carom uses the UMB 37 mm nose specification (Vector/Klematch P37 profile).
  // The remaining impact coefficients are explicitly a provisional carryover
  // from the standard pool calibration: manufacturer Shore hardness/rebound
  // resilience is not a ball-cushion coefficient of restitution.
  static const caromP37Baseline = CushionProfile(
    noseHeightMm: 37.0,
    restitution: 0.98,
    airborneRestitution: 0.82,
    pocketFacingRestitution: 0.82,
    slidingFriction: 0.14,
    tableImpactFriction: 0.20,
    lowSpeedFriction: 0.14,
    highSpeedFriction: 0.13,
    peakGripSpeed: 2.60,
    highSpeedReference: 4.00,
    sidespinGrip: 1.50,
    tangentialSpinTransfer: 0.55,
    tangentialRestitution: 0.44,
    highSpeedTangentialRestitution: 0.60,
    impactDurationMs: 7.0,
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
  Vec3 prevPos3;
  Vec3 prevVel3;
  Vec3 prevAngularVelocity;
  int prevBounceCount = 0;
  bool hadContactThisStep = false;
  bool active;
  bool pocketed;
  Vec2? pocketPosition;
  Set<String> activePocketSurfaces = const <String>{};
  final List<Vec2> pathPoints = <Vec2>[];

  final double radius;

  _Ball(this.id, Vec2 pos, {this.pocketed = false, this.radius = _R})
      : pos3 = Vec3(pos.x, pos.y, radius),
        vel3 = Vec3.zero,
        angularVelocity = Vec3.zero,
        prevPos3 = Vec3(pos.x, pos.y, radius),
        prevVel3 = Vec3.zero,
        prevAngularVelocity = Vec3.zero,
        active = !pocketed;

  Vec2 get pos => Vec2(pos3.x, pos3.y);

  set pos(Vec2 value) => pos3 = Vec3(value.x, value.y, pos3.z);

  Vec2 get vel => Vec2(vel3.x, vel3.y);

  set vel(Vec2 value) => vel3 = Vec3(value.x, value.y, vel3.z);

  Vec2 get spin =>
      Vec2(radius * angularVelocity.y, -radius * angularVelocity.x);

  set spin(Vec2 value) => angularVelocity = Vec3(
        -value.y / radius,
        value.x / radius,
        angularVelocity.z,
      );

  double get spinZ => angularVelocity.z;

  set spinZ(double value) =>
      angularVelocity = Vec3(angularVelocity.x, angularVelocity.y, value);

  bool get isMoving =>
      active &&
      (vel3.lengthSquared > 1.6e-7 || angularVelocity.lengthSquared > 4e-4);
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
/// [forSolver] omits replay frames and research-only trajectory diagnostics
/// while preserving the simulated physics and collision/path details.
PhysicsResult simulate({
  required List<PhysicsBall> balls,
  required String cueBallId,
  required ShotParams shot,
  required PhysicsTableConfig tableConfig,
  CushionProfile cushion = CushionProfile.standardPool,
  int shotIndex = 0,
  bool forPreview = false,
  bool forSolver = false,
  double? maxTimeSeconds,
  double ballBallFrictionScale = 1.0,
  double ballBallRestitution = _eBall,
}) {
  if (!ballBallFrictionScale.isFinite || ballBallFrictionScale < 0) {
    throw ArgumentError.value(
      ballBallFrictionScale,
      'ballBallFrictionScale',
      'must be finite and non-negative',
    );
  }
  if (!ballBallRestitution.isFinite ||
      ballBallRestitution < 0 ||
      ballBallRestitution > 1) {
    throw ArgumentError.value(
      ballBallRestitution,
      'ballBallRestitution',
      'must be finite and in [0, 1]',
    );
  }
  final dt = forPreview ? 0.003 : _dt;
  final recordEvery = forPreview ? 0.040 : _recordEvery;
  final maxTime = maxTimeSeconds ?? _maxTime;
  final cfg = _Cfg.from(tableConfig);

  final simBalls = balls
      .map(
        (b) => _Ball(
          b.id,
          _relToMeters(b.position, cfg),
          pocketed: b.pocketed,
          radius: cfg.radius,
        ),
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
    b.pathPoints.add(_metersToRel(b.pos, cfg));
    pathPoints[b.id] = b.pathPoints;
  }
  // Animation frames: all non-pocketed balls at each _recordEvery timestep.
  final frames = <Map<String, Vec2>>[];
  if (!forSolver) {
    frames.add({
      for (final b in simBalls)
        if (!b.pocketed) b.id: _metersToRel(b.pos, cfg),
    });
  }
  final peakHeights = forSolver
      ? <String, double>{}
      : <String, double>{for (final b in simBalls) b.id: b.pos3.z};
  final tableBounceCounts = <String, int>{for (final b in simBalls) b.id: 0};
  final tableBounceDistancesMeters = forSolver
      ? <String, List<double>>{}
      : <String, List<double>>{for (final b in simBalls) b.id: <double>[]};
  final tableBounceSpeedsMs = forSolver
      ? <String, List<double>>{}
      : <String, List<double>>{for (final b in simBalls) b.id: <double>[]};
  final tableBounceSpinRps = forSolver
      ? <String, List<double>>{}
      : <String, List<double>>{for (final b in simBalls) b.id: <double>[]};
  final horizontalDistanceMeters = forSolver
      ? <String, double>{}
      : <String, double>{for (final b in simBalls) b.id: 0.0};
  final peakSpinZ = forSolver
      ? <String, double>{}
      : <String, double>{for (final b in simBalls) b.id: b.spinZ.abs()};
  final peakSpinRpm = forSolver
      ? <String, double>{}
      : <String, double>{
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

    for (var i = 0; i < simBalls.length; i++) {
      final b = simBalls[i];
      b.prevPos3 = b.pos3;
      b.prevVel3 = b.vel3;
      b.prevAngularVelocity = b.angularVelocity;
      if (!forSolver) {
        b.prevBounceCount = tableBounceCounts[b.id]!;
      }
    }
    _applyFriction(
      simBalls,
      tableBounceCounts,
      cfg,
      forSolver ? null : sideSpinRatioAtRoll,
      dt,
    );
    final hitInfo = _resolveCollisions(
      simBalls,
      cueBallId,
      cfg,
      ballBallFrictionScale: ballBallFrictionScale,
      ballBallRestitution: ballBallRestitution,
      timeSec: t,
      timeStepSec: dt,
      contactDetails: ballBallContactDetails,
    );
    if (hitInfo != null && cueContactStep == null) {
      cueContactStep = step;
      ghostBallMeters = hitInfo;
    }
    if (!forSolver) {
      for (final b in simBalls) {
        final previous = b.prevPos3;
        final dx = b.pos3.x - previous.x;
        final dy = b.pos3.y - previous.y;
        horizontalDistanceMeters[b.id] =
            horizontalDistanceMeters[b.id]! + math.sqrt(dx * dx + dy * dy);
        if (tableBounceCounts[b.id]! > b.prevBounceCount) {
          tableBounceDistancesMeters[b.id]!.add(
            horizontalDistanceMeters[b.id]!,
          );
          tableBounceSpeedsMs[b.id]!.add(b.vel3.length);
          tableBounceSpinRps[b.id]!.add(
            b.angularVelocity.length / (2 * math.pi),
          );
        }
      }
    }
    final stepCushionContacts = _resolveCushions(
      simBalls,
      cfg,
      cushion,
      contactDetails: cushionContactDetails,
      timeSec: t,
    );
    cushionContacts += stepCushionContacts;
    for (final b in simBalls) {
      _limitAngularSpeed(b);
      if (b.hadContactThisStep) {
        b.hadContactThisStep = false;
        final point = _metersToRel(b.pos, cfg);
        final points = b.pathPoints;
        if (points.isEmpty || (points.last - point).distance > 1e-7) {
          points.add(point);
        }
      }
    }
    if (cfg.hasPockets) {
      final stepPocketContacts = _resolvePocketGeometry(
        simBalls,
        cfg,
        cushion,
        contactDetails: pocketContactDetails,
        timeSec: t,
      );
      if (stepPocketContacts > 0) {
        for (final b in simBalls) {
          if (b.hadContactThisStep) {
            b.hadContactThisStep = false;
            final point = _metersToRel(b.pos, cfg);
            final points = b.pathPoints;
            if (points.isEmpty || (points.last - point).distance > 1e-7) {
              points.add(point);
            }
          }
        }
      }
      _checkPockets(simBalls, cfg, cushion);
    }
    if (!forSolver) {
      for (final b in simBalls) {
        if (b.pos3.z > (peakHeights[b.id] ?? b.radius)) {
          peakHeights[b.id] = b.pos3.z;
        }
        if (b.spinZ.abs() > (peakSpinZ[b.id] ?? 0.0)) {
          peakSpinZ[b.id] = b.spinZ.abs();
        }
        final spinRpm = b.angularVelocity.length * _radiansPerSecondToRpm;
        if (spinRpm > (peakSpinRpm[b.id] ?? 0.0)) {
          peakSpinRpm[b.id] = spinRpm;
        }
      }
    }

    if (t >= nextRecord) {
      nextRecord += recordEvery;
      for (final b in simBalls) {
        if (b.isMoving) b.pathPoints.add(_metersToRel(b.pos, cfg));
      }
      if (!forSolver) {
        frames.add({
          for (final b in simBalls)
            if (!b.pocketed) b.id: _metersToRel(b.pos, cfg),
        });
      }
    }

    var anyMoving = false;
    for (var i = 0; i < simBalls.length; i++) {
      if (simBalls[i].isMoving) {
        anyMoving = true;
        break;
      }
    }
    if (!anyMoving) break;
  }

  // Preserve terminal state in the replay. A ball can be pocketed between
  // samples, then stop the simulation before the next regular frame is due.
  // Without this frame the animation freezes on the last pre-drop position.
  if (!forSolver) {
    final finalFrame = {
      for (final b in simBalls)
        if (!b.pocketed) b.id: _metersToRel(b.pos, cfg),
    };
    final lastFrame = frames.last;
    final frameChanged = lastFrame.length != finalFrame.length ||
        finalFrame.entries.any((entry) => lastFrame[entry.key] != entry.value);
    if (frameChanged) frames.add(finalFrame);
  }

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
      ? (cueContactStep * dt / recordEvery).floor().clamp(
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
        (b) => _Ball(
          b.id,
          _relToMeters(b.position, cfg),
          pocketed: b.pocketed,
          radius: cfg.radius,
        ),
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
    final clearanceDeg = _cueRailClearanceElevationDeg(
      tRail,
      hRail,
      rCue,
      cfg.radius,
    );
    elevDeg = math.max(elevDeg, clearanceDeg);
  }

  for (final ball in balls) {
    if (ball.id == cueBallId || !ball.active) continue;
    final diff = ball.pos - cbPos;

    final tBack = -(diff.x * aimDir.x + diff.y * aimDir.y);
    if (tBack < cfg.radius * 2 || tBack > lBridge + cfg.radius) continue;

    final axial = diff.x * aimDir.x + diff.y * aimDir.y;
    final perpX = diff.x - axial * aimDir.x;
    final perpY = diff.y - axial * aimDir.y;
    final lateral = math.sqrt(perpX * perpX + perpY * perpY);
    if (lateral >= cfg.radius + rCue) continue;

    final hBallTop = cfg.radius +
        (lateral < cfg.radius
            ? math.sqrt(cfg.radius * cfg.radius - lateral * lateral)
            : 0.0);
    final tanA = (hBallTop + rCue - cfg.radius) / tBack;
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
  double ballRadius,
) {
  double clearanceAt(double angle) {
    final alongCue = (distance - cueRadius * math.sin(angle)) / math.cos(angle);
    return ballRadius +
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
  cue.spinZ =
      -effectiveHitX * (cueStickVel / cue.radius) * _spinZFactor * cosElev;

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
  Map<String, double>? sideSpinRatioAtRoll,
  double dt,
) {
  final subSteps = (dt / _dt).round().clamp(1, 16);
  final subDt = dt / subSteps;
  final slipAccel = cfg.clothSlidingFriction * _g;
  final slipStep = 3.5 * slipAccel * subDt;
  final rollingDecel = cfg.clothRollingResistance * _g * subDt;
  final spinZDecay = spinZDecayRateRadPerSec2 * subDt;
  for (final b in balls) {
    if (!b.active || !b.isMoving) continue;

    // Exact closed-form integration across subSteps when grounded with no Z-spin
    // and no regime transition within this dt step.
    if (subSteps > 1 &&
        b.pos3.z <= b.radius + 1e-8 &&
        b.vel3.z == 0.0 &&
        b.spinZ.abs() <= 0.02) {
      final vcx = b.vel3.x - b.radius * b.angularVelocity.y;
      final vcy = b.vel3.y + b.radius * b.angularVelocity.x;
      final vcSpeedSq = vcx * vcx + vcy * vcy;
      if (vcSpeedSq <= 1.6e-7) {
        final speedSq = b.vel3.x * b.vel3.x + b.vel3.y * b.vel3.y;
        final totalRollingDecel = rollingDecel * subSteps;
        if (speedSq > (totalRollingDecel + 4e-4) * (totalRollingDecel + 4e-4)) {
          final speed = math.sqrt(speedSq);
          final posScale =
              (speed - 0.5 * (subSteps + 1) * rollingDecel) / speed * dt;
          final velScale = (speed - totalRollingDecel) / speed;
          b.pos3 = Vec3(
            b.pos3.x + b.vel3.x * posScale,
            b.pos3.y + b.vel3.y * posScale,
            b.radius,
          );
          final nextVel = Vec2(b.vel3.x * velScale, b.vel3.y * velScale);
          b.vel = nextVel;
          b.spin = nextVel;
          b.spinZ = 0.0;
          continue;
        }
      } else {
        final totalSlipStep = slipStep * subSteps;
        if (vcSpeedSq > totalSlipStep * totalSlipStep) {
          final vcSpeed = math.sqrt(vcSpeedSq);
          final invVc = 1.0 / vcSpeed;
          final vcDirX = vcx * invVc;
          final vcDirY = vcy * invVc;
          final posSlip =
              0.5 * subSteps * (subSteps + 1) * slipAccel * subDt * subDt;
          final velSlip = slipAccel * dt;
          final spinSlip = 2.5 * velSlip;
          final oldVx = b.vel3.x;
          final oldVy = b.vel3.y;
          final oldSpin = b.spin;
          b.pos3 = Vec3(
            b.pos3.x + oldVx * dt - vcDirX * posSlip,
            b.pos3.y + oldVy * dt - vcDirY * posSlip,
            b.radius,
          );
          b.vel = Vec2(oldVx - vcDirX * velSlip, oldVy - vcDirY * velSlip);
          b.spin = Vec2(
            oldSpin.x + vcDirX * spinSlip,
            oldSpin.y + vcDirY * spinSlip,
          );
          b.spinZ = 0.0;
          continue;
        }
      }
    }

    for (var step = 0; step < subSteps; step++) {
      if (!b.isMoving) break;

      final airborne = b.pos3.z > b.radius + 1e-8 || b.vel3.z > 0;
      if (airborne) {
        b.vel3 += Vec3(0, 0, -_g * subDt);
      }

      // Dr. Dave's high-speed-video analysis shows that a ball retains its spin
      // while airborne: cloth friction acts during table contact, not during a
      // hop. Apply the existing sliding/rolling model only while grounded.
      if (!airborne) {
        final vc = b.vel - b.spin;
        final vcSpeedSq = vc.x * vc.x + vc.y * vc.y;

        if (vcSpeedSq > 1.6e-7) {
          final vcSpeed = math.sqrt(vcSpeedSq);
          final vcDir = vc / vcSpeed;
          if (vcSpeed <= slipStep) {
            final rolling = b.vel - vcDir * (vcSpeed / 3.5);
            b.vel = rolling;
            b.spin = rolling;
            sideSpinRatioAtRoll?.putIfAbsent(
              b.id,
              () => b.vel.distance > 1e-9
                  ? b.spinZ.abs() * b.radius / b.vel.distance
                  : 0.0,
            );
          } else {
            b.vel = b.vel - vcDir * (slipAccel * subDt);
            b.spin = b.spin + vcDir * (2.5 * slipAccel * subDt);
          }
        } else {
          final speedSq = b.vel3.x * b.vel3.x + b.vel3.y * b.vel3.y;
          if (speedSq > 1.6e-7) {
            final speed = math.sqrt(speedSq);
            if (b.spinZ.abs() > 0.02) {
              // Dr. Dave TP B.2: the off-centre normal force that produces
              // rolling resistance also creates a small lateral force when the
              // ball has sidespin. Integrate its turn rate only in the rolling
              // regime; sliding and airborne motion use their own contact laws.
              final turnRate = (2.0 / 5.0) *
                  b.radius *
                  spinZDecayRateRadPerSec2 *
                  cfg.rollingTurnFactor /
                  speed;
              final spinDuration = math.min(
                subDt,
                b.spinZ.abs() / spinZDecayRateRadPerSec2,
              );
              final turnAngle = -b.spinZ.sign * turnRate * spinDuration;
              final cosTurn = math.cos(turnAngle);
              final sinTurn = math.sin(turnAngle);
              final turnedVelocity = Vec2(
                b.vel.x * cosTurn - b.vel.y * sinTurn,
                b.vel.x * sinTurn + b.vel.y * cosTurn,
              );
              if (speed <= rollingDecel) {
                b.vel = Vec2.zero;
                b.spin = Vec2.zero;
              } else {
                b.vel =
                    turnedVelocity - turnedVelocity.normalized * rollingDecel;
                b.spin = b.vel;
              }
            } else {
              if (speed <= rollingDecel) {
                b.vel = Vec2.zero;
                b.spin = Vec2.zero;
              } else {
                final scale = (speed - rollingDecel) / speed;
                b.vel = Vec2(b.vel3.x * scale, b.vel3.y * scale);
                b.spin = b.vel;
              }
            }
          } else {
            b.vel = Vec2.zero;
            b.spin = Vec2.zero;
          }
        }

        if (b.spinZ.abs() > 0.02) {
          b.spinZ = b.spinZ.abs() <= spinZDecay
              ? 0.0
              : b.spinZ - spinZDecay * b.spinZ.sign;
        } else {
          b.spinZ = 0.0;
        }

        if (b.spinZ.abs() < 0.02 &&
            (b.vel3.x * b.vel3.x + b.vel3.y * b.vel3.y) < 9e-8 &&
            (b.angularVelocity.x * b.angularVelocity.x +
                        b.angularVelocity.y * b.angularVelocity.y) *
                    (b.radius * b.radius) <
                9e-8) {
          b.vel = Vec2.zero;
          b.spin = Vec2.zero;
          b.spinZ = 0.0;
        }
      }

      b.pos3 += b.vel3 * subDt;
      if (b.pos3.z < b.radius) {
        b.pos3 = Vec3(b.pos3.x, b.pos3.y, b.radius);
        if (b.vel3.z < 0) {
          _resolveTableLanding(b, cfg.clothSlidingFriction);
          tableBounceCounts[b.id] = (tableBounceCounts[b.id] ?? 0) + 1;
        }
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
  final slipX = ball.vel3.x - ball.angularVelocity.y * ball.radius;
  final slipY = ball.vel3.y + ball.angularVelocity.x * ball.radius;
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
    impulseY / (_ballInertia * ball.radius),
    -impulseX / (_ballInertia * ball.radius),
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
  double durationSeconds,
  double ballBallRestitution,
) {
  if (balls.length < 3 || durationSeconds <= 0) {
    return const _HertzChainResolution(<String>{}, null);
  }

  // Fast path: a 3+ ball collinear contact chain requires at least 2 active
  // ball pairs to be touching or impacting during this timestep.
  var contactingPairs = 0;
  for (var i = 0; i < balls.length; i++) {
    final bi = balls[i];
    if (!bi.active) continue;
    final posI = bi.prevPos3;
    final touchDist = 2 * bi.radius + 1e-6;
    final touchDistSq = touchDist * touchDist;
    for (var j = i + 1; j < balls.length; j++) {
      final bj = balls[j];
      if (!bj.active) continue;
      final posJ = bj.prevPos3;
      final dx = posJ.x - posI.x;
      final dy = posJ.y - posI.y;
      final dz = posJ.z - posI.z;
      final touching = (dx * dx + dy * dy + dz * dz) <= touchDistSq;
      if (touching ||
          _ballBallTimeOfImpactFraction(
                posI,
                posJ,
                bi.pos3,
                bj.pos3,
                bi.radius,
              ) !=
              null) {
        contactingPairs++;
        if (contactingPairs >= 2) break;
      }
    }
    if (contactingPairs >= 2) break;
  }
  if (contactingPairs < 2) {
    return const _HertzChainResolution(<String>{}, null);
  }

  final unvisited = <int>{
    for (var i = 0; i < balls.length; i++)
      if (balls[i].active) i,
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
      final posI = balls[i].prevPos3;
      for (final j in unvisited.toList()) {
        final posJ = balls[j].prevPos3;
        final touching = (posJ - posI).length <= 2 * balls[i].radius + 1e-6;
        final willContact = _ballBallTimeOfImpactFraction(
          posI,
          posJ,
          balls[i].pos3,
          balls[j].pos3,
          balls[i].radius,
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
        final delta = balls[cluster[j]].prevPos3 - balls[cluster[i]].prevPos3;
        if (delta.lengthSquared > greatestDistanceSquared) {
          greatestDistanceSquared = delta.lengthSquared;
          firstIndex = cluster[i];
          lastIndex = cluster[j];
        }
      }
    }
    if (greatestDistanceSquared < 1e-12) continue;
    final axis =
        (balls[lastIndex].prevPos3 - balls[firstIndex].prevPos3).normalized();
    final ordered = cluster.toList()
      ..sort(
        (a, b) =>
            balls[a].prevPos3.dot(axis).compareTo(balls[b].prevPos3.dot(axis)),
      );
    final origin = balls[ordered.first].prevPos3;
    var collinear = true;
    var spinFree = true;
    for (final index in ordered) {
      final ball = balls[index];
      final positionDelta = ball.prevPos3 - origin;
      final positionResidual = positionDelta - axis * positionDelta.dot(axis);
      final velocity = ball.prevVel3;
      final velocityResidual = velocity - axis * velocity.dot(axis);
      if (positionResidual.length > 1e-6 || velocityResidual.length > 1e-4) {
        collinear = false;
        break;
      }
      if (ball.prevAngularVelocity.length > 1e-4) {
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
      final position = ball.prevPos3;
      final velocity = ball.prevVel3;
      initialPositions.add(position.dot(axis));
      initialVelocities.add(velocity.dot(axis));
      if (i == 0) continue;
      final gap = initialPositions[i] - initialPositions[i - 1];
      final left = balls[ordered[i - 1]];
      final right = balls[ordered[i]];
      final initiallyTouching = gap <= 2 * left.radius + 1e-6;
      final hasImpactThisStep = _ballBallTimeOfImpactFraction(
            left.prevPos3,
            right.prevPos3,
            left.pos3,
            right.pos3,
            left.radius,
          ) !=
          null;
      if (gap < 2 * left.radius - 0.002 ||
          (!initiallyTouching && !hasImpactThisStep)) {
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
      normalRestitution: ballBallRestitution,
    );
    Vec2? clusterCueContact;
    final cueIndex = ordered.indexWhere((i) => balls[i].id == cueBallId);
    if (cueIndex >= 0) {
      double? firstImpactFraction;
      for (final neighborIndex in [cueIndex - 1, cueIndex + 1]) {
        if (neighborIndex < 0 || neighborIndex >= ordered.length) continue;
        final cueBall = balls[ordered[cueIndex]];
        final neighbor = balls[ordered[neighborIndex]];
        final cueStart = cueBall.prevPos3;
        final neighborStart = neighbor.prevPos3;
        final fraction =
            (neighborStart - cueStart).length <= 2 * cueBall.radius + 1e-6
                ? 0.0
                : _ballBallTimeOfImpactFraction(
                    cueStart,
                    neighborStart,
                    cueBall.pos3,
                    neighbor.pos3,
                    cueBall.radius,
                  );
        if (fraction != null &&
            (firstImpactFraction == null || fraction < firstImpactFraction)) {
          firstImpactFraction = fraction;
        }
      }
      if (firstImpactFraction != null) {
        final cueBall = balls[ordered[cueIndex]];
        final start = cueBall.prevPos3;
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
      ball.angularVelocity = ball.prevAngularVelocity;
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
  double ballBallFrictionScale = 1.0,
  double ballBallRestitution = _eBall,
  double timeSec = 0.0,
  double timeStepSec = _dt,
  List<BallBallContact>? contactDetails,
}) {
  final hertz = _resolveHertzChains(
    balls,
    cueBallId,
    timeStepSec,
    ballBallRestitution,
  );
  Vec2? firstContact = hertz.cueContact;
  for (int i = 0; i < balls.length; i++) {
    final a = balls[i];
    if (!a.active) continue;
    for (int j = i + 1; j < balls.length; j++) {
      final b = balls[j];
      if (!b.active) continue;
      if (hertz.ballIds.isNotEmpty &&
          hertz.ballIds.contains(a.id) &&
          hertz.ballIds.contains(b.id)) {
        continue;
      }

      final previousA = a.prevPos3;
      final previousB = b.prevPos3;
      final currentA = a.pos3;
      final currentB = b.pos3;
      final timeOfImpact = _ballBallTimeOfImpactFraction(
        previousA,
        previousB,
        currentA,
        currentB,
        a.radius,
      );
      final impactA = timeOfImpact == null
          ? currentA
          : previousA + (currentA - previousA) * timeOfImpact;
      final impactB = timeOfImpact == null
          ? currentB
          : previousB + (currentB - previousB) * timeOfImpact;
      final dx = impactB.x - impactA.x;
      final dy = impactB.y - impactA.y;
      final dz = impactB.z - impactA.z;
      final distSq = dx * dx + dy * dy + dz * dz;
      final twoR = 2 * a.radius;
      if ((distSq < twoR * twoR || timeOfImpact != null) && distSq > 1e-12) {
        final dist = math.sqrt(distSq);
        final delta3 = Vec3(dx, dy, dz);
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
            (previousB - previousA).length <= 2 * a.radius + 1e-8;

        final overlap = 2 * a.radius - dist;
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
        final normalRestitution =
            isInitialFrozenCuePair ? 0.0 : ballBallRestitution;
        final jN = (1 + normalRestitution) * 0.5 * vN;
        a.vel3 -= n3 * jN;
        b.vel3 += n3 * jN;

        if (a.id == cueBallId || b.id == cueBallId) {
          firstContact ??= (a.id == cueBallId ? a : b).pos;
        }

        final rA = n3 * a.radius;
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
          a.angularVelocity +=
              rA.cross(impulse) / (_ballInertia * a.radius * a.radius);
          b.angularVelocity +=
              rB.cross(-impulse) / (_ballInertia * b.radius * b.radius);
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

double? _ballBallTimeOfImpactFraction(
  Vec3 a0,
  Vec3 b0,
  Vec3 a1,
  Vec3 b1, [
  double radius = _R,
]) {
  final sx = b0.x - a0.x;
  final sy = b0.y - a0.y;
  final sz = b0.z - a0.z;
  final dx = (b1.x - a1.x) - sx;
  final dy = (b1.y - a1.y) - sy;
  final dz = (b1.z - a1.z) - sz;
  final bb = 2 * (sx * dx + sy * dy + sz * dz);
  if (bb >= 0) return null;
  final aa = dx * dx + dy * dy + dz * dz;
  final cc = (sx * sx + sy * sy + sz * sz) - 4 * radius * radius;
  if (aa < 1e-18 || cc < -1e-12) return null;

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
  if (b.pos3.z <= b.radius + 1e-6 && b.vel3.z.abs() <= 1e-5) {
    _applyGroundedCushionImpact(b, normal, profile);
    return;
  }

  _applyLegacyCushionImpact(
    b,
    normal,
    tangent,
    profile,
    profile.airborneRestitution,
    includeVerticalFriction: b.pos3.z > b.radius + 0.001,
  );
}

void _applyGroundedCushionImpact(
  _Ball b,
  Vec3 inwardNormal,
  CushionProfile profile,
) {
  const mass = _cueBallMassKg;
  const slipTolerance = 4e-4;
  final sinTheta = ((profile.noseHeightMm / 1000.0 - b.radius) / b.radius)
      .clamp(0.05, 0.95)
      .toDouble();
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
    final slipRailX = vx + b.radius * (wy * sinTheta - wz * cosTheta);
    final slipRailY = -vy * sinTheta + wx * b.radius;
    final slipRail = math.sqrt(slipRailX * slipRailX + slipRailY * slipRailY);
    final hasRailSlip = slipRail > slipTolerance;
    final phi = hasRailSlip ? math.atan2(slipRailY, slipRailX) : 0.0;

    final slipTableX = vx - wy * b.radius;
    final slipTableY = vy + wx * b.radius;
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
    final dwx =
        -2.5 / (mass * b.radius) * (muRailY + muTableY * tableNormalFactor);
    final dwy = -2.5 /
        (mass * b.radius) *
        (muRailX * sinTheta - muTableX * tableNormalFactor);
    final dwz = 2.5 / (mass * b.radius) * muRailX * cosTheta;

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
  final noseOffset = profile.noseHeightMm / 1000.0 - b.radius;
  final r = normal * -b.radius + Vec3(0, 0, noseOffset);
  final inertia = _ballInertia * b.radius * b.radius;
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
    final spinRatio =
        (b.spinZ.abs() * b.radius) / math.max(b.vel3.length, 1e-6);
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
    b.radius,
  );
  b.vel3 -= tangent * (tangentVelocity * (1.0 - speedRetention));
  b.spinZ *= speedRetention;
}

double _cushionTangentialRetention(
  CushionProfile profile,
  double incidentSpeed,
  double spinZ,
  double radius,
) {
  final t = _smoothStep((incidentSpeed - 2.60) / (13.4112 - 2.60));
  final base = profile.tangentialRestitution +
      (profile.highSpeedTangentialRestitution - profile.tangentialRestitution) *
          t;
  final spinRatio = (spinZ.abs() * radius) / math.max(incidentSpeed, 1e-6);
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

    if (b.pos3.x < b.radius && _railFacePresentAtEdge('left', b.pos, cfg)) {
      b.pos = Vec2(b.radius, b.pos3.y);
      if (-b.vel3.x > 0) {
        contacts++;
        b.hadContactThisStep = true;
        impact(b, const Vec3(1, 0, 0), const Vec3(0, 1, 0), 'left');
      }
    }

    if (b.pos3.x > cfg.W - b.radius &&
        _railFacePresentAtEdge('right', b.pos, cfg)) {
      b.pos = Vec2(cfg.W - b.radius, b.pos3.y);
      if (b.vel3.x > 0) {
        contacts++;
        b.hadContactThisStep = true;
        impact(b, const Vec3(-1, 0, 0), const Vec3(0, 1, 0), 'right');
      }
    }

    if (b.pos3.y < b.radius && _railFacePresentAtEdge('top', b.pos, cfg)) {
      b.pos = Vec2(b.pos3.x, b.radius);
      if (-b.vel3.y > 0) {
        contacts++;
        b.hadContactThisStep = true;
        impact(b, const Vec3(0, 1, 0), const Vec3(1, 0, 0), 'top');
      }
    }

    if (b.pos3.y > cfg.H - b.radius &&
        _railFacePresentAtEdge('bottom', b.pos, cfg)) {
      b.pos = Vec2(b.pos3.x, cfg.H - b.radius);
      if (b.vel3.y > 0) {
        contacts++;
        b.hadContactThisStep = true;
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
  if (!cfg.hasPockets) return true;
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

// The ball's centre drops as soon as it enters the rendered black hole. The
// facings can still reject it before that boundary; once its centre crosses
// the circle, the ball is no longer supported by the shelf.
_CornerFrame? _capturingCorner(Vec2 pos, _Cfg cfg) {
  final captureRadiusSq = cfg.cornerHoleR * cfg.cornerHoleR;
  for (final corner in cfg.cornerFrames) {
    final dx = pos.x - corner.holeCenter.x;
    final dy = pos.y - corner.holeCenter.y;
    if (dx * dx + dy * dy < captureRadiusSq) return corner;
  }
  return null;
}

String? _capturingSidePocket(Vec2 pos, _Cfg cfg) {
  final captureRadiusSq = cfg.sideHoleR * cfg.sideHoleR;
  final dx = pos.x - cfg.W / 2;
  if (dx * dx >= captureRadiusSq) return null;
  final dyTop = pos.y + cfg.sideHoleOffset;
  if (dx * dx + dyTop * dyTop < captureRadiusSq) return 'topSide';
  final dyBottom = pos.y - (cfg.H + cfg.sideHoleOffset);
  if (dx * dx + dyBottom * dyBottom < captureRadiusSq) return 'bottomSide';
  return null;
}

bool _onCornerShelfOrBetweenJaws(Vec2 pos, _Cfg cfg) {
  if (_capturingCorner(pos, cfg) != null) return false;
  final leg = cfg.cornerJawR;
  final outsideLimit = cfg.cornerHoleR;
  for (final corner in cfg.cornerFrames) {
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

Vec2 _closestPointOnSegment(Vec2 point, Vec2 start, Vec2 end) {
  final sx = end.x - start.x;
  final sy = end.y - start.y;
  final lengthSquared = sx * sx + sy * sy;
  if (lengthSquared < 1e-16) return start;
  final ox = point.x - start.x;
  final oy = point.y - start.y;
  final t = ((ox * sx + oy * sy) / lengthSquared).clamp(0.0, 1.0);
  return Vec2(start.x + sx * t, start.y + sy * t);
}

// Resolve the full corner-pocket and side-pocket facing segments.
// Each segment naturally includes its jaw endpoint, retaining a rounded
// collision at the cushion tip while enforcing the angled neoprene facing.
int _resolvePocketGeometry(
  List<_Ball> balls,
  _Cfg cfg,
  CushionProfile profile, {
  List<PocketContact>? contactDetails,
  double timeSec = 0.0,
}) {
  var contacts = 0;
  final facings = cfg.pocketFacings;
  final margin = cfg.pocketCheckMargin;

  for (final b in balls) {
    if (!b.active) continue;
    final bx = b.pos3.x;
    final by = b.pos3.y;
    if (bx >= margin &&
        bx <= cfg.W - margin &&
        by >= margin &&
        by <= cfg.H - margin) {
      if (b.activePocketSurfaces.isNotEmpty) {
        b.activePocketSurfaces = const <String>{};
      }
      continue;
    }

    Set<String>? ballSurfaces;
    final r = b.radius;
    for (var i = 0; i < facings.length; i++) {
      final facing = facings[i];
      if (bx < facing.minX - r ||
          bx > facing.maxX + r ||
          by < facing.minY - r ||
          by > facing.maxY + r) {
        continue;
      }
      final closest = _closestPointOnSegment(b.pos, facing.start, facing.end);
      final dx = b.pos3.x - closest.x;
      final dy = b.pos3.y - closest.y;
      final distSq = dx * dx + dy * dy;
      if (distSq >= r * r || distSq < 1e-16) continue;
      final distance = math.sqrt(distSq);
      final normal2 = Vec2(dx / distance, dy / distance);
      final normal3 = Vec3(normal2.x, normal2.y, 0);
      if (-b.vel3.dot(normal3) <= 0) continue;

      final surfaceKey = '${facing.pocket}/${facing.surface}';
      (ballSurfaces ??= <String>{}).add(surfaceKey);
      final isNewContact = !b.activePocketSurfaces.contains(surfaceKey);

      final incidentSpeed = b.vel3.length;
      b.pos = closest + normal2 * r;
      final tangent2 = facing.direction;
      _applyLegacyCushionImpact(
        b,
        normal3,
        Vec3(tangent2.x, tangent2.y, 0),
        profile,
        profile.pocketFacingRestitution,
        includeVerticalFriction: false,
      );
      _limitAngularSpeed(b);
      contacts++;
      b.hadContactThisStep = true;
      if (isNewContact) {
        contactDetails?.add(
          PocketContact(
            timeSec: timeSec,
            ballId: b.id,
            pocket: facing.pocket,
            surface: facing.surface,
            positionRel: _metersToRel(b.pos, cfg),
            incidentSpeedMs: incidentSpeed,
            reboundSpeedMs: b.vel3.length,
          ),
        );
      }
    }
    b.activePocketSurfaces = ballSurfaces ?? const <String>{};
  }

  return contacts;
}

void _checkPockets(List<_Ball> balls, _Cfg cfg, CushionProfile profile) {
  final margin = cfg.pocketCheckMargin;
  // The back of every pocket is the rail cap. A ball whose centre is over the
  // hole and whose lowest point is below the cap either drops straight in or
  // strikes the back of the pocket and then drops, so capture does not require
  // cloth contact. Only a ball high enough to clear the rail cap escapes.
  final pocketBackHeight =
      profile.noseHeightMm / 1000.0 + railCapAboveNoseInches * inchesToMeters;
  for (final b in balls) {
    if (!b.active) continue;
    final bx = b.pos3.x;
    final by = b.pos3.y;
    if (bx >= margin &&
        bx <= cfg.W - margin &&
        by >= margin &&
        by <= cfg.H - margin) {
      continue;
    }
    if (b.pos3.z - b.radius >= pocketBackHeight) continue;
    // Both corner and side pockets require the ball's centre to cross the
    // pocket hole circle past the shelf; the facings can reject it before then.
    if (_capturingCorner(b.pos, cfg) != null ||
        _capturingSidePocket(b.pos, cfg) != null) {
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
