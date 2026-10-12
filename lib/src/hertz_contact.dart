import 'dart:math' as math;

/// Result of integrating a one-dimensional, initially touching ball chain.
/// Positions are centre coordinates along the chain axis, in metres.
class HertzChainState {
  final List<double> positions;
  final List<double> velocities;

  const HertzChainState({required this.positions, required this.velocities});
}

const double _ballRadiusM = 0.028575;
const double _ballMassKg = 0.170;
const double _hertzCompliancePerNewton = 7.266e-7;
const double _contactDamping = 5234.6245;
const double _microStepSeconds = 1e-6;

/// Integrates a collinear chain of initially touching equal pool balls.
///
/// This is a finite-duration Hertz normal-contact model based on Dr. Dave's
/// TP B.29. [normalRestitution] is represented by a calibrated viscoelastic
/// dashpot (`0.93` at a 1 m/s isolated-ball impact), with a `v^(-1/5)` scale
/// to preserve the configured restitution across impact speeds. Use `1.0` for
/// B.29's ideal-elastic reference case. Tangential contact/friction is
/// deliberately outside this 1-D helper and remains handled by the simulator's
/// existing collision path for non-chain contacts.
HertzChainState integrateHertzContactChain({
  required List<double> positions,
  required List<double> velocities,
  required double durationSeconds,
  double normalRestitution = 0.93,
}) {
  if (positions.length != velocities.length || positions.length < 2) {
    throw ArgumentError(
      'A Hertz chain needs matching arrays of at least 2 balls.',
    );
  }
  if (durationSeconds < 0 || normalRestitution < 0 || normalRestitution > 1) {
    throw ArgumentError('Invalid duration or normal restitution.');
  }

  var x = List<double>.of(positions);
  var v = List<double>.of(velocities);
  var elapsed = 0.0;
  final referenceSpeed = [
    for (var i = 0; i < velocities.length - 1; i++)
      math.max(0.0, velocities[i] - velocities[i + 1]),
  ].fold(0.0, math.max);
  // For Hertz contact, peak compression scales approximately as v^(4/5), so
  // the dashpot/elastic force ratio scales as v^(1/5). Compensate with v^(-1/5)
  // to preserve the engine's speed-independent restitution parameter.
  final speedScale = referenceSpeed > 1e-9
      ? math.pow(1.0 / referenceSpeed, 0.2).toDouble()
      : 1.0;
  final damping =
      _contactDamping * (1 - normalRestitution) / (1 - 0.93) * speedScale;

  List<double> acceleration(List<double> currentX, List<double> currentV) {
    final forces = List<double>.filled(currentX.length, 0.0);
    for (var i = 0; i < currentX.length - 1; i++) {
      final compression = 2 * _ballRadiusM - (currentX[i + 1] - currentX[i]);
      if (compression <= 0) continue;
      final closingSpeed = currentV[i] - currentV[i + 1];
      final elasticForce = math.pow(2 * compression / _ballRadiusM, 1.5) /
          _hertzCompliancePerNewton;
      final dampingForce = damping * math.sqrt(compression) * closingSpeed;
      final force = math.max(0.0, elasticForce + dampingForce);
      forces[i] -= force;
      forces[i + 1] += force;
    }
    return forces.map((force) => force / _ballMassKg).toList();
  }

  while (elapsed < durationSeconds) {
    final step = math.min(_microStepSeconds, durationSeconds - elapsed);
    final a0 = acceleration(x, v);
    final nextX = [
      for (var i = 0; i < x.length; i++)
        x[i] + v[i] * step + 0.5 * a0[i] * step * step,
    ];
    final predictedV = [for (var i = 0; i < v.length; i++) v[i] + a0[i] * step];
    final a1 = acceleration(nextX, predictedV);
    v = [
      for (var i = 0; i < v.length; i++) v[i] + 0.5 * (a0[i] + a1[i]) * step,
    ];
    x = nextX;
    elapsed += step;
  }
  return HertzChainState(positions: x, velocities: v);
}
