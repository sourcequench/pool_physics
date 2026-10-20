import 'package:pool_physics/pool_physics.dart';
import 'package:test/test.dart';

void main() {
  const table = PhysicsTableConfig.nineFootDiamond;
  const shot = ShotParams(aimDir: Vec2(1, 0), speedMs: 2.5, hitX: 0, hitY: 0);

  List<PhysicsBall> balls() => const [
        PhysicsBall(id: 'cue', x: 0.30, y: 0.50),
        PhysicsBall(id: 'object', x: 0.45, y: 0.50),
      ];

  test(
    'solver output mode preserves physics but skips diagnostic payloads',
    () {
      final full = simulate(
        balls: balls(),
        cueBallId: 'cue',
        shot: shot,
        tableConfig: table,
      );
      final solver = simulate(
        balls: balls(),
        cueBallId: 'cue',
        shot: shot,
        tableConfig: table,
        forSolver: true,
      );

      expect(solver.endPositions, full.endPositions);
      expect(solver.cushionContacts, full.cushionContacts);
      expect(solver.ballBallContactDetails, isNotEmpty);
      expect(
        solver.ballBallContactDetails.length,
        full.ballBallContactDetails.length,
      );
      expect(
        solver.paths.map((path) => path.points),
        full.paths.map((path) => path.points),
      );
      expect(solver.frames, isEmpty);
      expect(solver.peakHeights, isEmpty);
      expect(solver.peakSpinRpm, isEmpty);
      expect(solver.peakSpinZ, isEmpty);
      expect(solver.tableBounceDistancesMeters, isEmpty);
      expect(solver.tableBounceSpeedsMs, isEmpty);
      expect(solver.tableBounceSpinRps, isEmpty);
      expect(solver.sideSpinRatioAtRoll, isEmpty);
    },
  );

  test(
    'ball-ball restitution override reaches discrete and Hertz collisions',
    () {
      final standard = simulate(
        balls: balls(),
        cueBallId: 'cue',
        shot: shot,
        tableConfig: table,
      );
      final elastic = simulate(
        balls: balls(),
        cueBallId: 'cue',
        shot: shot,
        tableConfig: table,
        ballBallRestitution: 1,
      );

      expect(standard.ballBallContactDetails, isNotEmpty);
      expect(elastic.ballBallContactDetails, isNotEmpty);
      expect(
        elastic.ballBallContactDetails.first.normalImpulsePerMass,
        greaterThan(
          standard.ballBallContactDetails.first.normalImpulsePerMass,
        ),
      );
    },
  );

  test('ball-ball restitution must be finite and in [0, 1]', () {
    for (final invalid in [-0.01, 1.01, double.nan, double.infinity]) {
      expect(
        () => simulate(
          balls: balls(),
          cueBallId: 'cue',
          shot: shot,
          tableConfig: table,
          ballBallRestitution: invalid,
        ),
        throwsArgumentError,
      );
    }
  });

  test(
    'forPreview integrates friction at 3 ms steps until rest and matches 1 ms end positions on long shots',
    () {
      const rack = <PhysicsBall>[
        PhysicsBall(id: 'cue', x: 0.8872, y: 0.4622),
        PhysicsBall(id: 'ball-1', x: 0.1634, y: 0.2118),
        PhysicsBall(id: 'ball-2', x: 0.1825, y: 0.3250),
        PhysicsBall(id: 'ball-3', x: 0.2976, y: 0.6783),
      ];
      const longShot = ShotParams(
        aimDir: Vec2(-0.9833, -0.1821),
        speedMs: 3.3528,
        hitX: 0,
        hitY: 0,
      );

      final normal = simulate(
        balls: rack,
        cueBallId: 'cue',
        shot: longShot,
        tableConfig: table,
        forPreview: false,
      );
      final preview = simulate(
        balls: rack,
        cueBallId: 'cue',
        shot: longShot,
        tableConfig: table,
        forPreview: true,
      );

      expect(preview.pocketPositions.keys, normal.pocketPositions.keys);
      expect(preview.finalLinearVelocities['cue']!.length, lessThan(1e-3));
      final normalCueEnd = normal.endPositions['cue']!;
      final previewCueEnd = preview.endPositions['cue']!;
      expect((previewCueEnd - normalCueEnd).distance, lessThan(0.02));
      expect(
        (preview.ballBallContactDetails.first.timeSec -
                normal.ballBallContactDetails.first.timeSec)
            .abs(),
        lessThan(0.01),
      );
    },
  );
}
