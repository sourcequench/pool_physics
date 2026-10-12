import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

const _radius = 0.028575;
const _mass = 0.170;
const _hertzCompliance = 7.266e-7; // 1/N, TP B.29 from Marlow's data
const _impactStep = 2.84e-8; // TP B.29 uses contact duration / 10,000

class _ChainState {
  final List<double> positions;
  final List<double> velocities;
  final List<double> accelerations;

  const _ChainState(this.positions, this.velocities, this.accelerations);
}

List<double> _accelerations(List<double> positions) {
  final forces = [0.0, 0.0, 0.0];
  for (var i = 0; i < 2; i++) {
    final compression = 2 * _radius - (positions[i + 1] - positions[i]);
    if (compression <= 0) continue;

    // Hertz contact: F = (1/c) * (2δ/R)^(3/2). The two adjacent balls
    // receive equal-and-opposite forces, so linear momentum is conserved.
    final force = math.pow(2 * compression / _radius, 1.5) / _hertzCompliance;
    forces[i] -= force;
    forces[i + 1] += force;
  }
  return forces.map((force) => force / _mass).toList();
}

_ChainState _advance(_ChainState state) {
  final positions = [
    for (var i = 0; i < 3; i++)
      state.positions[i] +
          state.velocities[i] * _impactStep +
          0.5 * state.accelerations[i] * _impactStep * _impactStep,
  ];
  final accelerations = _accelerations(positions);
  final velocities = [
    for (var i = 0; i < 3; i++)
      state.velocities[i] +
          0.5 * (state.accelerations[i] + accelerations[i]) * _impactStep,
  ];
  return _ChainState(positions, velocities, accelerations);
}

void main() {
  test('TP B.29 elastic Hertz chain reproduces reported velocity ratios', () {
    var state = _ChainState([
      0,
      2 * _radius,
      4 * _radius
    ], [
      1,
      0,
      0
    ], [
      0,
      0,
      0,
    ]);
    var elapsed = 0.0;

    // Run past the reported 0.503 ms three-ball contact duration, until the
    // balls have separated, matching the source proof's integration scheme.
    for (var i = 0; i < 40000; i++) {
      state = _advance(state);
      elapsed += _impactStep;
      final separated =
          state.positions[1] - state.positions[0] >= 2 * _radius &&
              state.positions[2] - state.positions[1] >= 2 * _radius;
      if (elapsed > 0.001 && separated) break;
    }

    expect(elapsed, greaterThan(0.0005));
    expect(state.velocities[0], closeTo(-0.071, 0.001));
    expect(state.velocities[1], closeTo(0.076, 0.001));
    expect(state.velocities[2], closeTo(0.995, 0.001));
    expect(state.velocities.reduce((a, b) => a + b), closeTo(1, 1e-6));
    expect(
      state.velocities.map((v) => v * v).reduce((a, b) => a + b),
      closeTo(1, 1e-5),
    );
  });

  test('finite-contact chain solver matches the elastic B.29 reference', () {
    final result = integrateHertzContactChain(
      positions: [0, 2 * _radius, 4 * _radius],
      velocities: [1, 0, 0],
      durationSeconds: 0.001,
      normalRestitution: 1,
    );
    expect(result.velocities[0], closeTo(-0.071, 0.001));
    expect(result.velocities[1], closeTo(0.076, 0.001));
    expect(result.velocities[2], closeTo(0.995, 0.001));
  });

  test('viscoelastic extension preserves e=0.93 across shot speeds', () {
    for (final speed in [0.44704, 1.0, 1.34112, 3.12928, 7.0]) {
      final result = integrateHertzContactChain(
        positions: [0, 2 * _radius],
        velocities: [speed, 0],
        durationSeconds: 0.005,
      );
      expect(
        (result.velocities[1] - result.velocities[0]) / speed,
        closeTo(0.93, 0.001),
        reason: 'impact speed $speed m/s',
      );
      expect(result.velocities.reduce((a, b) => a + b), closeTo(speed, 1e-6));
    }
  });

  test('simulator uses finite contact for a touching three-ball chain', () {
    const tableW = 2.540;
    const tableH = 1.270;
    const rx0 = 7 / 114;
    const ry0 = 7 / 64;
    const spanX = 1 - 14 / 114;
    const spanY = 1 - 14 / 64;
    Vec2 rel(Vec2 p) =>
        Vec2(p.x / tableW * spanX + rx0, p.y / tableH * spanY + ry0);

    // Keep >57 in of clear cue length behind the cue ball so the geometry
    // model does not add elevation to avoid the rail.
    final start = const Vec2(1.8, 0.635);
    final balls = [
      for (var i = 0; i < 3; i++)
        PhysicsBall(
          id: 'b$i',
          x: rel(Vec2(start.x + 2 * _radius * i, start.y)).x,
          y: rel(Vec2(start.x + 2 * _radius * i, start.y)).y,
        ),
    ];
    final result = simulate(
      balls: balls,
      cueBallId: 'b0',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: 1,
        elevationDeg: 0,
      ),
      tableConfig: const PhysicsTableConfig(
        playW: tableW,
        playH: tableH,
        minimumCueElevationDeg: 0,
        clothSlidingFriction: 0,
        clothRollingResistance: 0,
      ),
      maxTimeSeconds: 0.001,
    );
    final velocities = result.finalLinearVelocities;

    // The production profile retains the engine's e=0.93 calibration. This
    // independent damped-Hertz reference keeps momentum while allowing the
    // expected energy loss, and—unlike sequential impulses—reverses the cue.
    expect(velocities['b0']!.x, closeTo(-0.04838, 0.001));
    expect(velocities['b1']!.x, closeTo(0.10967, 0.001));
    expect(velocities['b2']!.x, closeTo(0.93871, 0.001));
    expect(
      velocities.values.map((v) => v.x).reduce((a, b) => a + b),
      closeTo(1, 1e-5),
    );
    expect(velocities['b0']!.x, lessThan(0));
    expect(
      result.ballBallContactDetails,
      isEmpty,
      reason: 'a simultaneous chain is not a single cue/object impulse',
    );
  });

  test('finite-contact chain is detected when the cue first reaches it', () {
    const tableW = 2.540;
    const tableH = 1.270;
    const rx0 = 7 / 114;
    const ry0 = 7 / 64;
    const spanX = 1 - 14 / 114;
    const spanY = 1 - 14 / 64;
    Vec2 rel(Vec2 p) =>
        Vec2(p.x / tableW * spanX + rx0, p.y / tableH * spanY + ry0);
    Vec2 metres(Vec2 p) =>
        Vec2((p.x - rx0) / spanX * tableW, (p.y - ry0) / spanY * tableH);

    const cueStartX = 1.8;
    const firstObjectX = cueStartX + 2 * _radius + 0.010;
    const y = 0.635;
    final result = simulate(
      balls: [
        for (final entry in [
          ('cue', cueStartX, y),
          ('ob1', firstObjectX, y),
          ('ob2', firstObjectX + 2 * _radius, y),
        ])
          PhysicsBall(
            id: entry.$1,
            x: rel(Vec2(entry.$2, entry.$3)).x,
            y: rel(Vec2(entry.$2, entry.$3)).y,
          ),
      ],
      cueBallId: 'cue',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: 1,
        elevationDeg: 0,
      ),
      tableConfig: const PhysicsTableConfig(
        playW: tableW,
        playH: tableH,
        minimumCueElevationDeg: 0,
        clothSlidingFriction: 0,
        clothRollingResistance: 0,
      ),
      // The cue travels 10 mm before the touching object-ball pair. This
      // checks swept-cluster detection, not only an impact at t=0.
      maxTimeSeconds: 0.011,
    );
    expect(result.finalLinearVelocities['cue']!.x, closeTo(-0.04838, 0.002));
    expect(result.finalLinearVelocities['ob1']!.x, closeTo(0.10967, 0.002));
    expect(result.finalLinearVelocities['ob2']!.x, closeTo(0.93871, 0.002));
    expect(result.finalLinearVelocities['cue']!.x, lessThan(0));
    expect(
      metres(result.ghostBallRel!).x,
      closeTo(firstObjectX - 2 * _radius, 1e-4),
    );
  });
}
