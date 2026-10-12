import 'dart:math' as math;

/// Cue geometry used for the deterministic TP A.22 baseline profile: Dr. Dave's
/// average 12.5 mm tip with a nickel-radius dome (0.835 in diameter).
const double referenceCueTipDiameterM = 0.0125;
const double referenceCueTipCurvatureRadiusM = 0.835 * 0.0254 / 2;
const double standardPoolBallRadiusM = 0.028575;

/// Vertical clearance between the cue tip and cloth for a low (draw-side)
/// contact at [offsetR] ball radii from center. Positive clearance means the
/// idealized spherical tip does not penetrate the playing surface.
///
/// This implements the tip/table geometry in Dr. Dave TP A.22. It is separate
/// from the TP 2.1 cue-tip friction/miscue limit. [cueElevationDeg] is the
/// cue's elevation above horizontal; tip geometry is specified in metres.
double cueTipTableClearanceM({
  required double offsetR,
  required double cueElevationDeg,
  double ballRadiusM = standardPoolBallRadiusM,
  double tipDiameterM = referenceCueTipDiameterM,
  double tipCurvatureRadiusM = referenceCueTipCurvatureRadiusM,
}) {
  if (!offsetR.isFinite ||
      offsetR < 0 ||
      offsetR >= 1 ||
      !cueElevationDeg.isFinite ||
      cueElevationDeg < 0 ||
      cueElevationDeg >= 90 ||
      !ballRadiusM.isFinite ||
      ballRadiusM <= 0 ||
      !tipDiameterM.isFinite ||
      tipDiameterM <= 0 ||
      !tipCurvatureRadiusM.isFinite ||
      tipCurvatureRadiusM < tipDiameterM / 2) {
    throw ArgumentError('Invalid cue/table clearance geometry.');
  }

  final tipHalfAngle = math.asin(tipDiameterM / (2 * tipCurvatureRadiusM));
  final contactAngle = math.acos(offsetR.clamp(0.0, 1.0));
  final elevation = cueElevationDeg * math.pi / 180;
  return ballRadiusM -
      (ballRadiusM + tipCurvatureRadiusM) * math.cos(elevation + contactAngle) -
      tipCurvatureRadiusM * math.sin(elevation + tipHalfAngle);
}

/// Largest low/draw-side offset that clears the cloth for the cue geometry and
/// elevation (TP A.22). This is not itself the miscue limit; callers must also
/// enforce the cue-tip friction limit from TP 2.1.
double maximumCueDrawOffsetForTableClearanceR({
  required double cueElevationDeg,
  double ballRadiusM = standardPoolBallRadiusM,
  double tipDiameterM = referenceCueTipDiameterM,
  double tipCurvatureRadiusM = referenceCueTipCurvatureRadiusM,
}) {
  double clearance(double offsetR) => cueTipTableClearanceM(
        offsetR: offsetR,
        cueElevationDeg: cueElevationDeg,
        ballRadiusM: ballRadiusM,
        tipDiameterM: tipDiameterM,
        tipCurvatureRadiusM: tipCurvatureRadiusM,
      );

  if (clearance(0) < 0) return 0;
  var low = 0.0;
  var high = 1.0 - 1e-12;
  for (var i = 0; i < 60; i++) {
    final mid = (low + high) / 2;
    if (clearance(mid) >= 0) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return low;
}

/// Constrains a normalized vertical tip offset by the independent TP 2.1
/// friction/miscue limit and TP A.22 table-clearance limit. Follow offsets are
/// unchanged because the cue tip is above the ball center on that side.
double constrainCueDrawOffsetR({
  required double requestedOffsetR,
  required double cueElevationDeg,
  double maximumMiscueOffsetR = 0.5,
  double ballRadiusM = standardPoolBallRadiusM,
  double tipDiameterM = referenceCueTipDiameterM,
  double tipCurvatureRadiusM = referenceCueTipCurvatureRadiusM,
}) {
  if (!requestedOffsetR.isFinite ||
      !maximumMiscueOffsetR.isFinite ||
      maximumMiscueOffsetR < 0) {
    throw ArgumentError('Invalid cue tip offset.');
  }
  if (requestedOffsetR >= 0) return requestedOffsetR;
  final maximumDrawR = math.min(
    maximumMiscueOffsetR,
    maximumCueDrawOffsetForTableClearanceR(
      cueElevationDeg: cueElevationDeg,
      ballRadiusM: ballRadiusM,
      tipDiameterM: tipDiameterM,
      tipCurvatureRadiusM: tipCurvatureRadiusM,
    ),
  );
  return math.max(requestedOffsetR, -maximumDrawR);
}

/// Solid-sphere angular/linear surface-speed ratio per normalized tip offset,
/// from TP A.30's angular impulse relation (also TP A.12).
const double cueBallSpinRatioPerOffset = 2.5;

double cueBallSpinRatioFromTipOffset(double tipOffsetR) =>
    cueBallSpinRatioPerOffset * tipOffsetR;

/// Minimum tangential cue-tip friction coefficient needed to avoid slip at
/// [offsetR], expressed as a fraction of cue-ball radius.
///
/// This is Dr. Dave's TP 2.1 no-slip boundary. An offset at or beyond the
/// equator has no finite solution in this idealized horizontal-impact model.
double requiredCueTipFrictionForOffset(double offsetR) {
  final offset = offsetR.abs();
  if (!offset.isFinite || offset >= 1) {
    throw ArgumentError.value(offsetR, 'offsetR', 'must be finite and < 1R');
  }
  return offset / math.sqrt(1 - offset * offset);
}

/// Largest normalized tip offset that can be transmitted without slip for a
/// given cue-tip/ball friction coefficient (TP 2.1).
double maximumNoSlipCueTipOffsetR(double frictionCoefficient) {
  if (!frictionCoefficient.isFinite || frictionCoefficient < 0) {
    throw ArgumentError.value(
      frictionCoefficient,
      'frictionCoefficient',
      'must be finite and non-negative',
    );
  }
  return frictionCoefficient /
      math.sqrt(1 + frictionCoefficient * frictionCoefficient);
}

/// Cue-ball / cue-stick speed ratio for an offset hit, using TP A.30's
/// conservation-of-momentum and collision-efficiency equations.
///
/// [ballToCueMassRatio] is ball mass / effective cue mass. The higher root is
/// the separating-impact solution. [energyEfficiency] is the fraction of
/// incoming cue kinetic energy transferred to cue-ball translation and spin.
double cueBallSpeedRatioFromCueImpact({
  required double tipOffsetR,
  double ballToCueMassRatio = 6 / 19,
  double energyEfficiency = 0.87,
}) {
  if (!tipOffsetR.isFinite ||
      ballToCueMassRatio <= 0 ||
      !ballToCueMassRatio.isFinite ||
      energyEfficiency < 0 ||
      energyEfficiency > 1 ||
      !energyEfficiency.isFinite) {
    throw ArgumentError('Invalid cue impact parameters.');
  }
  final offsetSquared = tipOffsetR * tipOffsetR;
  final quadratic =
      ballToCueMassRatio * (1 + ballToCueMassRatio + 2.5 * offsetSquared);
  final discriminant = ballToCueMassRatio * ballToCueMassRatio -
      quadratic * (1 - energyEfficiency);
  if (discriminant < 0) {
    throw ArgumentError(
      'No separating cue impact exists for these offset/efficiency values.',
    );
  }
  return (ballToCueMassRatio + math.sqrt(discriminant)) / quadratic;
}

/// TP A.30 relation between a centre-hit cue COR and collision energy
/// efficiency for the specified ball-to-cue mass ratio.
double cueImpactEfficiencyFromRestitution({
  required double restitution,
  double ballToCueMassRatio = 6 / 19,
}) {
  if (restitution < 0 ||
      restitution > 1 ||
      !restitution.isFinite ||
      ballToCueMassRatio <= 0 ||
      !ballToCueMassRatio.isFinite) {
    throw ArgumentError('Invalid cue impact parameters.');
  }
  final ratio = ballToCueMassRatio;
  return ((1 - ratio * restitution) * (1 - ratio * restitution) +
          ratio * (1 + restitution) * (1 + restitution)) /
      ((1 + ratio) * (1 + ratio));
}
