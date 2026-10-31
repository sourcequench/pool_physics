import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart' as sim;
import 'package:pool_physics/pool_physics.dart';

void main() {
  test('solver output mode preserves physics and omits presentation data', () {
    const balls = [
      PhysicsBall(id: 'cue', x: 0.5, y: 0.7),
      PhysicsBall(id: 'ob', x: 0.5, y: 0.5),
    ];
    const shot = sim.ShotParams(
      aimDir: Vec2(0, -1),
      speedMs: 2.4,
      hitX: 0.2,
      hitY: 0.1,
    );

    final detailed = sim.simulate(
      balls: balls,
      cueBallId: 'cue',
      shot: shot,
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );
    final solver = sim.simulate(
      balls: balls,
      cueBallId: 'cue',
      shot: shot,
      tableConfig: PhysicsTableConfig.nineFootDiamond,
      forSolver: true,
    );

    expect(solver.endPositions, detailed.endPositions);
    expect(solver.pocketPositions, detailed.pocketPositions);
    expect(solver.paths.length, detailed.paths.length);
    for (var i = 0; i < solver.paths.length; i++) {
      expect(solver.paths[i].ballId, detailed.paths[i].ballId);
      expect(solver.paths[i].style, detailed.paths[i].style);
      expect(solver.paths[i].points, detailed.paths[i].points);
    }
    expect(
      solver.cushionContactDetails.length,
      detailed.cushionContactDetails.length,
    );
    expect(
      solver.pocketContactDetails.length,
      detailed.pocketContactDetails.length,
    );
    expect(
      solver.ballBallContactDetails.length,
      detailed.ballBallContactDetails.length,
    );
    expect(solver.tableBounceCounts, detailed.tableBounceCounts);
    expect(solver.endHeights, detailed.endHeights);
    expect(solver.finalLinearVelocities, detailed.finalLinearVelocities);
    expect(solver.frames, isEmpty);
    expect(solver.peakHeights, isEmpty);
    expect(solver.tableBounceDistancesMeters, isEmpty);
    expect(solver.tableBounceSpeedsMs, isEmpty);
    expect(solver.tableBounceSpinRps, isEmpty);
    expect(solver.peakSpinZ, isEmpty);
    expect(solver.peakSpinRpm, isEmpty);
    expect(solver.sideSpinRatioAtRoll, isEmpty);
  });
}
