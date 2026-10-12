// Demonstrates a straight-in follow shot on a nine-foot table.
//
// Run with:  dart run example/straight_shot.dart

// ignore_for_file: avoid_print

import 'package:pool_physics/pool_physics.dart';

String _fmt(Vec2? v) => v == null
    ? 'pocketed'
    : '(${v.x.toStringAsFixed(3)}, ${v.y.toStringAsFixed(3)})';

void main() {
  // Place the cue ball a quarter of the way up the table and the nine ball
  // three-quarters of the way up, both on the centre line.
  final result = simulate(
    balls: [
      PhysicsBall.at('cue', const Vec2(0.25, 0.50)),
      PhysicsBall.at('nine', const Vec2(0.75, 0.50)),
    ],
    cueBallId: 'cue',
    shot: const ShotParams(
      aimDir: Vec2(1, 0), // shoot toward the right rail
      speedMs: 2.0, // 2 m/s ≈ 4.5 mph — medium speed
      hitX: 0.0, // centre horizontally
      hitY: 0.35, // slight follow (above centre)
    ),
    tableConfig: PhysicsTableConfig.nineFootDiamond,
  );

  // Where did everything end up?
  print('Cue ball:  ${_fmt(result.endPositions['cue'])}');
  print('Nine ball: ${_fmt(result.endPositions['nine'])}');
  print('Cushion contacts: ${result.cushionContacts}');

  // Inspect path segments.
  for (final path in result.paths) {
    print(
      '  ${path.ballId} [${path.style.name}]: '
      '${path.points.length} samples, '
      '${_fmt(path.points.first)} → '
      '${_fmt(path.points.last)}',
    );
  }

  // Peak spin for the cue ball (TP A.12: SRF = Rω/V = 2.5 × tip offset).
  final peakRpm = result.peakSpinRpm['cue'] ?? 0.0;
  print('Peak cue spin: ${peakRpm.toStringAsFixed(0)} RPM');

  // If there was a ball–ball contact, print its throw angle.
  if (result.ballBallContactDetails.isNotEmpty) {
    final contact = result.ballBallContactDetails.first;
    const radToDeg = 180.0 / 3.14159265358979;
    print(
      'Throw angle: '
      '${(contact.throwAngleRad * radToDeg).toStringAsFixed(2)}°',
    );
  }
}
