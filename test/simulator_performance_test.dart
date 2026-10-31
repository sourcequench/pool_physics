import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart' as sim;
import 'package:pool_physics/pool_physics.dart';

void main() {
  test('solver-mode simulation performance sample', () {
    const balls = <PhysicsBall>[
      PhysicsBall(id: 'cue', x: 0.50, y: 0.78),
      PhysicsBall(id: 'one', x: 0.50, y: 0.48),
      PhysicsBall(id: 'two', x: 0.72, y: 0.30),
      PhysicsBall(id: 'three', x: 0.25, y: 0.26),
    ];
    const shot = sim.ShotParams(
      aimDir: Vec2(0.03, -0.9995498987),
      speedMs: 4.2,
      hitX: -0.22,
      hitY: 0.38,
    );

    final watch = Stopwatch()..start();
    var checksum = 0.0;
    const iterations = 20;
    for (var i = 0; i < iterations; i++) {
      final result = sim.simulate(
        balls: balls,
        cueBallId: 'cue',
        shot: shot,
        tableConfig: PhysicsTableConfig.nineFootDiamond,
        forPreview: true,
        forSolver: true,
      );
      final end = result.endPositions['cue'];
      if (end != null) checksum += math.sqrt(end.x * end.x + end.y * end.y);
    }
    watch.stop();
    // Printed duration is intentionally informational; no wall-clock threshold
    // makes this repeatable across developer machines and CI hosts.
    // ignore: avoid_print
    print(
      'solver-sim benchmark: $iterations shots in '
      '${watch.elapsedMilliseconds}ms (checksum ${checksum.toStringAsFixed(3)})',
    );
    expect(checksum, greaterThan(0));
  });
}
