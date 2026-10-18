import 'package:pool_physics/pool_physics.dart';
import 'package:test/test.dart';

void main() {
  test('carom table uses 61.5 mm balls and four continuous cushions', () {
    const config = PhysicsTableConfig(
      playW: 2.84,
      playH: 1.42,
      ballRadiusM: 0.03075,
      hasPockets: false,
      relX0: 0.05,
      relX1: 0.95,
      relY0: 0.1,
      relY1: 0.9,
    );
    const cue = PhysicsBall(id: 'cue', x: 0.5, y: 0.5);

    final result = simulate(
      balls: const [cue],
      cueBallId: 'cue',
      shot: const ShotParams(aimDir: Vec2(-1, 0), speedMs: 2.0),
      tableConfig: config,
      cushion: CushionProfile.caromP37Baseline,
    );

    final leftRail = result.cushionContactDetails
        .where((contact) => contact.rail == 'left')
        .first;
    final expectedX = config.relX0 +
        config.ballRadiusM / config.playW * (config.relX1 - config.relX0);
    expect(leftRail.positionRel.x, closeTo(expectedX, 0.002));
    expect(result.pocketPositions, isEmpty);
  });

  test('carom fast-cloth lag baseline rolls near the published 0.55% slope',
      () {
    const config = PhysicsTableConfig(
      playW: 50,
      playH: 25,
      ballRadiusM: 0.03075,
      hasPockets: false,
      relX0: 0.05,
      relX1: 0.95,
      relY0: 0.1,
      relY1: 0.9,
      clothRollingResistance: 0.0055,
    );
    final result = simulate(
      balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
      cueBallId: 'cue',
      shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 1.0),
      tableConfig: config,
      cushion: CushionProfile.caromP37Baseline,
    );
    final endX = result.endPositions['cue']!.x;
    final travel = (endX - 0.5) / (config.relX1 - config.relX0) * config.playW;

    expect(travel, inInclusiveRange(4.2, 5.5));
  });
}
