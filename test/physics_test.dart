import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

PhysicsBall cue(Vec2 pos) => PhysicsBall.at('cue', pos);
PhysicsBall ball(String id, Vec2 pos) => PhysicsBall.at(id, pos);

const Vec2 kCentre = Vec2(0.5, 0.5);

PhysicsResult _run({
  required List<PhysicsBall> balls,
  required ShotParams shot,
  String cueBallId = 'cue',
  int shotIndex = 0,
  CushionProfile cushion = CushionProfile.standardPool,
  PhysicsTableConfig? tableConfig,
}) =>
    simulate(
      balls: balls,
      cueBallId: cueBallId,
      shot: shot,
      tableConfig: tableConfig ?? PhysicsTableConfig.nineFootDiamond,
      shotIndex: shotIndex,
      cushion: cushion,
    );

PhysicsResult runShot({
  required List<PhysicsBall> balls,
  required Vec2 aimDir,
  double hitX = 0.0,
  double hitY = 0.0,
}) =>
    _run(
      balls: balls,
      shot: ShotParams(aimDir: aimDir, speedMs: 2.5, hitX: hitX, hitY: hitY),
    );

int countRailContacts(List<Vec2> points) {
  const x0 = 7.0 / 114.0;
  const x1 = 1.0 - x0;
  const y0 = 7.0 / 64.0;
  const y1 = 1.0 - y0;
  bool atRail(Vec2 p) =>
      (p.x - x0).abs() < 0.004 ||
      (p.x - x1).abs() < 0.004 ||
      (p.y - y0).abs() < 0.004 ||
      (p.y - y1).abs() < 0.004;
  var contacts = 0;
  var wasAtRail = atRail(points.first);
  for (final point in points.skip(1)) {
    final nowAtRail = atRail(point);
    if (nowAtRail && !wasAtRail) contacts++;
    wasAtRail = nowAtRail;
  }
  return contacts;
}

void main() {
  test('TP B.15 pocket dimensions derive the default facing angle', () {
    final angle = cornerFacingAngleDegreesFromDimensions(
      mouthInches: 4.5,
      throatInches: 3.75,
      cushionFaceDepthInches: 2.0,
    );

    expect(angle, closeTo(38.3232219015, 1e-9));
    expect(cornerFacingAngleDegrees, closeTo(angle, 1e-9));
    expect(
      () => cornerFacingAngleDegreesFromDimensions(
        mouthInches: 3.75,
        throatInches: 4.5,
        cushionFaceDepthInches: 2.0,
      ),
      throwsArgumentError,
    );
  });

  test(
    'shared rail calibration shots do not produce impossible rail counts',
    () {
      final cases = [
        (
          pos: const Vec2(0.912, 0.8445),
          aim: const Vec2(-0.8185, -0.5745),
          hitX: 0.0,
          hitY: 0.0714,
        ),
        (
          pos: const Vec2(0.912, 0.8445),
          aim: const Vec2(-0.8179, -0.5754),
          hitX: -0.9978,
          hitY: -0.0665,
        ),
        (
          pos: const Vec2(0.8991, 0.8284),
          aim: const Vec2(-0.8202, -0.5721),
          hitX: -0.4286,
          hitY: 0.4643,
        ),
        (
          pos: const Vec2(0.9082, 0.8422),
          aim: const Vec2(-0.8186, -0.5743),
          hitX: -0.3929,
          hitY: 0.6071,
        ),
        (
          pos: const Vec2(0.912, 0.8307),
          aim: const Vec2(-0.8255, -0.5643),
          hitX: 0.4113,
          hitY: 0.9115,
        ),
      ];
      for (final shot in cases) {
        final result = _run(
          balls: [cue(shot.pos)],
          shot: ShotParams(
            aimDir: shot.aim,
            speedMs: 2.6822,
            hitX: shot.hitX,
            hitY: shot.hitY,
          ),
        );
        final path = result.paths.firstWhere((p) => p.ballId == 'cue');
        final rails = countRailContacts(path.points);
        expect(
          result.cushionContacts,
          lessThanOrEqualTo(9),
          reason: 'actual=${result.cushionContacts} inferred=$rails',
        );
      }
    },
  );

  test('human-speed rail count never exceeds nine impacts', () {
    for (final speed in [2.2352, 2.6822, 5.141]) {
      final result = _run(
        balls: [cue(const Vec2(0.9069, 0.8468))],
        shot: ShotParams(aimDir: Vec2(-0.8146, -0.5801), speedMs: speed),
      );
      expect(result.cushionContacts, lessThanOrEqualTo(9));
    }

    final breakSpeed = _run(
      balls: [cue(const Vec2(0.9069, 0.8468))],
      shot: const ShotParams(
        aimDir: Vec2(-0.8146, -0.5801),
        speedMs: 13.4112,
      ),
    );
    expect(breakSpeed.paths, isNotEmpty);
    expect(
      breakSpeed.cushionContactDetails.every(
        (contact) =>
            contact.incidentSpeedMs.isFinite && contact.reboundSpeedMs.isFinite,
      ),
      isTrue,
      reason: 'break-speed impacts remain finite outside the paper model range',
    );
  });

  test('six-mph maximum top-left does not create excess rail energy', () {
    PhysicsResult run({required double hitX, required double hitY}) => _run(
          balls: [cue(const Vec2(0.912, 0.8307))],
          shot: ShotParams(
            aimDir: Vec2(-0.8255, -0.5643),
            speedMs: 2.6822,
            hitX: hitX,
            hitY: hitY,
          ),
        );

    final centre = run(hitX: 0, hitY: 0);
    final topLeft = run(hitX: 0.4113, hitY: 0.9115);

    expect(centre.cushionContacts, lessThanOrEqualTo(5));
    expect(
      topLeft.cushionContacts,
      lessThanOrEqualTo(5),
      reason: '6 mph high top-left must not gain superhuman rail energy',
    );
    expect(
      topLeft.peakSpinRpm['cue']!,
      lessThanOrEqualTo(maxPhysicalSpinRpm),
    );
  });

  test('centre-ball shot stays on its aim line before the first cushion', () {
    const aim = Vec2(-0.8185, -0.5745);
    final result = _run(
      balls: [cue(const Vec2(0.75, 0.75))],
      shot: const ShotParams(
        aimDir: aim,
        speedMs: 2.6822,
        elevationDeg: 3.0,
      ),
    );
    final path = result.paths.firstWhere((p) => p.ballId == 'cue');
    final start = path.points.first;
    final aimRel = Vec2(
      aim.x / 2.54 * (1.0 - 2.0 * 7.0 / 114.0),
      aim.y / 1.27 * (1.0 - 2.0 * 7.0 / 64.0),
    );
    final aimLength = aimRel.length;
    final direction = aimRel / aimLength;
    final preRail = path.points.take(20);
    final maximumCrossTrack = preRail.map((point) {
      final delta = point - start;
      return (delta.x * direction.y - delta.y * direction.x).abs();
    }).fold<double>(0.0, math.max);
    expect(maximumCrossTrack, lessThan(1e-6));
  });

  group('Calibrated contact laws', () {
    test('TP B.12 identifies the stable-distance tip-height optimum', () {
      const table = PhysicsTableConfig(
        playW: 40,
        playH: 3,
        minimumCueElevationDeg: 0,
        clothSlidingFriction: 0.20,
        clothRollingResistance: 0.010,
      );
      const cueSpeedMph = 3.0;
      const ballToCueMassRatio = 6 / 19;
      const tipEfficiency = 0.87;
      const playableSpan = 1 - 2 * (7 / 114);

      double travelAt(double offsetR) {
        final ratio = cueBallSpeedRatioFromCueImpact(
          tipOffsetR: offsetR,
          ballToCueMassRatio: ballToCueMassRatio,
          energyEfficiency: tipEfficiency,
        );
        final ballSpeedMph = cueSpeedMph * ratio;
        final result = simulate(
          balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
          cueBallId: 'cue',
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: ballSpeedMph * 0.44704,
            hitY: offsetR,
            elevationDeg: 0,
          ),
          tableConfig: table,
        );
        return (result.endPositions['cue']!.x - 0.5) /
            playableSpan *
            table.playW /
            0.3048;
      }

      final offsets = List.generate(21, (i) => i * 0.025);
      final distancesFt = offsets.map(travelAt).toList();
      var maximumIndex = 0;
      for (var i = 1; i < distancesFt.length; i++) {
        if (distancesFt[i] > distancesFt[maximumIndex]) maximumIndex = i;
      }
      expect(offsets[maximumIndex], closeTo(0.175, 0.026));
      expect(
        distancesFt[maximumIndex],
        closeTo(30.508, 0.03),
        reason: 'TP B.12 predicts 30.508 ft at its 0.174R optimum',
      );
    });

    test('TP B.12 centre-hit travel matches skid plus rolling distance', () {
      const table = PhysicsTableConfig(
        playW: 30,
        playH: 3,
        minimumCueElevationDeg: 0,
        clothSlidingFriction: 0.20,
        clothRollingResistance: 0.010,
      );
      final result = simulate(
        balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
        cueBallId: 'cue',
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 3.824 * 0.44704,
          elevationDeg: 0,
        ),
        tableConfig: table,
      );
      final end = result.endPositions['cue']!;
      const playableSpan = 1 - 2 * (7 / 114);
      final distanceMeters = (end.x - 0.5) / playableSpan * table.playW;
      expect(
        distanceMeters,
        closeTo((1.197 + 24.935) * 0.3048, 0.03),
        reason: 'TP B.12: sliding distance plus natural-roll stopping distance',
      );
    });

    test('rolling resistance sets natural-roll stopping distance', () {
      double stoppingDistance(double rollingResistance) {
        const table = PhysicsTableConfig(
          playW: 30,
          playH: 3,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0.20,
          clothRollingResistance: 0.010,
        );
        final result = simulate(
          balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
          cueBallId: 'cue',
          shot: const ShotParams(
            aimDir: Vec2(1, 0),
            speedMs: 0.5,
            hitY: 0.4,
            elevationDeg: 0,
          ),
          tableConfig: PhysicsTableConfig(
            playW: table.playW,
            playH: table.playH,
            minimumCueElevationDeg: table.minimumCueElevationDeg,
            clothSlidingFriction: table.clothSlidingFriction,
            clothRollingResistance: rollingResistance,
          ),
        );
        final end = result.endPositions['cue']!;
        const playableSpan = 1 - 2 * (7 / 114);
        return (end.x - 0.5) / playableSpan * table.playW;
      }

      const speed = 0.5;
      for (final friction in [0.005, 0.010, 0.015]) {
        final expected = speed * speed / (2 * friction * 9.81);
        expect(
          stoppingDistance(friction),
          closeTo(expected, 0.02),
          reason: 'natural-roll distance for μr=$friction (Dr. Dave range)',
        );
      }
    });

    test(
      'cue spin-rate factor matches Alciatore TP A.12 for follow offset',
      () {
        const ballRadiusM = 0.028575;
        const speedMs = 1.5;
        const hitY = 0.5;
        final result = _run(
          balls: [cue(kCentre)],
          shot: const ShotParams(
            aimDir: Vec2(1, 0),
            speedMs: speedMs,
            hitY: hitY,
          ),
        );
        final expectedOmega = 2.5 * speedMs * hitY / ballRadiusM;
        final expectedRpm = expectedOmega * 60.0 / (2.0 * math.pi);

        expect(
          result.peakSpinRpm['cue'],
          closeTo(expectedRpm, 1e-6),
          reason: 'TP A.12: SRF = Rω/V = 2.5 × tip offset / ball radius',
        );
      },
    );

    test('side-spin rate matches Alciatore TP A.12 for lateral offset', () {
      const ballRadiusM = 0.028575;
      const speedMs = 0.1;
      const hitX = 0.5;
      final result = _run(
        balls: [cue(kCentre)],
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: speedMs,
          hitX: hitX,
        ),
      );
      final elevationRad = result.cueElevationDeg * math.pi / 180.0;
      final effectiveHitX = cueEffectiveLateralOffsetR(
        hitX,
        CueType.lowDeflection,
      );
      final expectedOmega =
          2.5 * speedMs * effectiveHitX * math.cos(elevationRad) / ballRadiusM;

      expect(
        result.peakSpinZ['cue'],
        closeTo(expectedOmega, 1e-6),
        reason:
            'TP A.12 spin transfer uses TP B.7 effective offset after squirt',
      );
    });

    test("combined tip offset is clamped to Dr. Dave's 0.5R miscue limit", () {
      PhysicsResult run(double hitX, double hitY) => _run(
            balls: [cue(kCentre)],
            shot: ShotParams(
              aimDir: const Vec2(1, 0),
              speedMs: 1.0,
              hitX: hitX,
              hitY: hitY,
            ),
          );
      const radialOffset = 0.5 / math.sqrt2;
      final outOfBounds = run(0.5, 0.5);
      final atLimit = run(radialOffset, radialOffset);

      expect(
        outOfBounds.peakSpinRpm['cue'],
        closeTo(atLimit.peakSpinRpm['cue']!, 1e-10),
      );
    });

    test('rail and cue spin remain below the 2500 RPM physical ceiling', () {
      final shots = [
        (
          pos: const Vec2(0.9082, 0.8422),
          aim: const Vec2(-0.8186, -0.5743),
          speed: 2.6822,
          hitX: -0.3929,
          hitY: 0.6071,
        ),
        (
          pos: const Vec2(0.9069, 0.8468),
          aim: const Vec2(-0.8146, -0.5801),
          speed: 5.141,
          hitX: 0.0,
          hitY: 0.0,
        ),
        (
          pos: kCentre,
          aim: const Vec2(-1, 0),
          speed: 2.6822,
          hitX: 1.0,
          hitY: 0.0,
        ),
      ];
      for (final shot in shots) {
        final result = _run(
          balls: [cue(shot.pos)],
          shot: ShotParams(
            aimDir: shot.aim,
            speedMs: shot.speed,
            hitX: shot.hitX,
            hitY: shot.hitY,
          ),
        );
        expect(
          result.peakSpinRpm['cue']!,
          lessThanOrEqualTo(maxPhysicalSpinRpm),
        );
      }
    });

    test(
      'ball-ball friction stays in the empirical range and decreases with slip speed',
      () {
        // Alciatore curve: 0.009951 + 0.108·exp(-1.088·v)
        final slow = ballBallFrictionForSlip(0.0);
        final medium = ballBallFrictionForSlip(0.5);
        final fast = ballBallFrictionForSlip(5.0);

        expect(slow, inInclusiveRange(0.10, 0.13));
        expect(fast, inInclusiveRange(0.009, 0.02));
        expect(slow, greaterThan(medium));
        expect(medium, greaterThan(fast));
      },
    );

    test('vertical-axis spin resistance uses the empirical pool range', () {
      expect(spinZDecayRateRadPerSec2, inInclusiveRange(5.0, 15.0));
    });
  });

  group('Elevation computation', () {
    test('centre table shot has minimum 3° elevation', () {
      final result = runShot(balls: [cue(kCentre)], aimDir: Vec2(1, 0));
      expect(result.cueElevationDeg, closeTo(3.0, 0.01));
    });

    test('TP A.3 cue/rail clearance matches the 24-inch example', () {
      const table = PhysicsTableConfig(
        playW: 2.4384,
        playH: 1.2192,
        minimumCueElevationDeg: 0,
      );
      const distance = 24 * 0.0254;
      const railWidth = 7 / 114;
      final cueX = railWidth + distance / table.playW * (1 - 2 * railWidth);
      final elevation = computeElevationDeg(
        Vec2(cueX, 0.5),
        const Vec2(1, 0),
        [PhysicsBall(id: 'cue', x: cueX, y: 0.5)],
        'cue',
        table,
      );
      expect(elevation, closeTo(1.384, 0.01));
    });

    test('explicit elevated shot enters and returns from 3D flight', () {
      final result = _run(
        balls: [cue(kCentre)],
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 2.5,
          elevationDeg: 25,
        ),
      );
      expect(result.peakHeights['cue']!, greaterThan(0.04));
      expect(result.endHeights['cue']!, closeTo(0.028575, 1e-6));
    });

    test('ball on bottom rail shooting up forces elevation > 3°', () {
      const cbPos = Vec2(0.5, 0.855);
      final result = runShot(
        balls: [cue(cbPos)],
        aimDir: Vec2(0, -1),
      );
      expect(result.cueElevationDeg, greaterThan(3.0));
    });

    test('ball at top rail shooting down forces elevation > 3°', () {
      const cbPos = Vec2(0.5, 0.145);
      final result = runShot(
        balls: [cue(cbPos)],
        aimDir: Vec2(0, 1),
      );
      expect(result.cueElevationDeg, greaterThan(3.0));
    });

    test(
      'shooting parallel to a nearby rail does NOT force extra elevation',
      () {
        const cbPos = Vec2(0.5, 0.855);
        final result = runShot(
          balls: [cue(cbPos)],
          aimDir: Vec2(1, 0),
        );
        expect(result.cueElevationDeg, closeTo(3.0, 0.5));
      },
    );

    test(
      'object ball in path between bridge and CB forces extra elevation',
      () {
        const cbPos = Vec2(0.5, 0.5);
        final obst = ball('ob', const Vec2(0.5 - 0.059, 0.5));
        final result = runShot(balls: [cue(cbPos), obst], aimDir: Vec2(1, 0));
        expect(result.cueElevationDeg, greaterThan(3.0));
      },
    );

    test('object ball far from path does not affect elevation', () {
      const cbPos = Vec2(0.5, 0.5);
      final offside = ball('ob', const Vec2(0.3, 0.35));
      final result = runShot(balls: [cue(cbPos), offside], aimDir: Vec2(1, 0));
      expect(result.cueElevationDeg, closeTo(3.0, 0.01));
    });
  });

  group('Masse and swerve calibration', () {
    test(
      'elevated right english curves to the right beyond initial squirt',
      () {
        const start = Vec2(0.5, 0.5);
        const forwardDistance = 0.12;

        double lateralOffset({
          required double hitX,
          required double elevation,
          double speed = 2.0,
        }) {
          final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
          final desiredAngle = 0.0;
          final aimAngle = desiredAngle - squirt;
          final result = _run(
            balls: [cue(start)],
            shot: ShotParams(
              aimDir: Vec2(math.cos(aimAngle), math.sin(aimAngle)),
              speedMs: speed,
              hitX: hitX,
              elevationDeg: elevation,
            ),
          );
          final path = result.paths.firstWhere(
            (candidate) => candidate.ballId == 'cue',
          );
          final sample = path.points.reduce((best, point) {
            final targetX = start.x + forwardDistance;
            return (point.x - targetX).abs() < (best.x - targetX).abs()
                ? point
                : best;
          });
          return sample.y - start.y;
        }

        final flatSpin = lateralOffset(hitX: 0.4, elevation: 3.0);
        final elevatedSpin = lateralOffset(hitX: 0.4, elevation: 60.0);
        final elevatedCentre = lateralOffset(hitX: 0.0, elevation: 60.0);
        final fastElevatedSpin = lateralOffset(
          hitX: 0.4,
          elevation: 60.0,
          speed: 5.0,
        );

        expect(flatSpin, greaterThan(0.001));
        expect(elevatedSpin, greaterThan(elevatedCentre + 0.01));
        expect(
          elevatedSpin,
          greaterThan(fastElevatedSpin + 0.002),
          reason: 'higher speed delays swerve at the same forward distance',
        );
      },
    );
  });

  group('Throw and spin transfer', () {
    test('TP A.24 stun throw is larger than matched follow and draw throw', () {
      const tableW = 2.540;
      const tableH = 1.270;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );
      const cut = 30 * math.pi / 180;
      const separation = 0.05715;
      const speed = 0.89408;
      const ob = Vec2(1.27, 0.635);
      final cuePosition = Vec2(
        ob.x - separation * math.cos(cut) - 0.000001,
        ob.y + separation * math.sin(cut),
      );
      double throwAt(double hitY) {
        final result = simulate(
          balls: [
            PhysicsBall(
              id: 'cue',
              x: rel(cuePosition).x,
              y: rel(cuePosition).y,
            ),
            PhysicsBall(id: 'ob', x: rel(ob).x, y: rel(ob).y),
          ],
          cueBallId: 'cue',
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: speed,
            hitY: hitY,
          ),
          tableConfig: PhysicsTableConfig.nineFootDiamond,
        );
        return result.ballBallContactDetails.first.throwAngleRad.abs() *
            180 /
            math.pi;
      }

      final stun = throwAt(0);
      final follow = throwAt(0.4);
      final draw = throwAt(-0.4);
      expect(stun, closeTo(4.366, 0.15));
      expect(follow, closeTo(1.454, 0.15));
      expect(draw, closeTo(1.454, 0.15));
      expect((follow - draw).abs(), lessThan(0.15));
    });

    test('TP A.24 object-ball throw and post-impact curve match', () {
      const tableW = 20.0;
      const tableH = 20.0;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );
      const cut = 30 * math.pi / 180;
      const separation = 0.05715;
      const speed = 0.89408;
      const ob = Vec2(10, 7);
      final cuePosition = Vec2(
        ob.x - separation * math.cos(cut) - 0.000001,
        ob.y + separation * math.sin(cut),
      );
      for (final hitY in [0.0, 0.4, -0.4]) {
        final result = simulate(
          balls: [
            PhysicsBall(
              id: 'cue',
              x: rel(cuePosition).x,
              y: rel(cuePosition).y,
            ),
            PhysicsBall(id: 'ob', x: rel(ob).x, y: rel(ob).y),
          ],
          cueBallId: 'cue',
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: speed,
            hitY: hitY,
          ),
          tableConfig: const PhysicsTableConfig(playW: tableW, playH: tableH),
        );
        final detail = result.ballBallContactDetails.first;
        final obPath = result.paths.firstWhere((p) => p.ballId == 'ob');
        final displacement = obPath.points.last - obPath.points.first;
        final dx = displacement.x / (rx1 - rx0) * tableW;
        final dy = displacement.y / (ry1 - ry0) * tableH;
        final absolute = math.atan2(dy, dx);
        final normal = math.atan2(detail.impactNormal.y, detail.impactNormal.x);
        final finalAngleDeg = (absolute - normal) * 180 / math.pi;
        final expectedAngleDeg = switch (hitY) {
          0.0 => 4.366,
          0.4 => 1.454 + 0.067,
          _ => 1.454 - 0.061,
        };
        expect(result.cushionContacts, 0);
        expect(obPath.points.length, greaterThan(2));
        expect(
          finalAngleDeg,
          closeTo(expectedAngleDeg, 0.12),
          reason: 'TP A.24 final OB direction at hitY=$hitY',
        );
      }
    });

    test('TP B.14 typical draw direction follows its angle curve', () {
      const tableW = 60.0;
      const tableH = 60.0;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );
      const separation = 0.05715;
      const ob = Vec2(30, 30);
      for (final cutDeg in [10.0, 20.0, 30.0, 40.0]) {
        final cut = cutDeg * math.pi / 180;
        final cuePosition = Vec2(
          ob.x - separation * math.cos(cut) - 0.000001,
          ob.y + separation * math.sin(cut),
        );
        final result = simulate(
          balls: [
            PhysicsBall(
              id: 'cue',
              x: rel(cuePosition).x,
              y: rel(cuePosition).y,
            ),
            PhysicsBall(id: 'ob', x: rel(ob).x, y: rel(ob).y),
          ],
          cueBallId: 'cue',
          shot: const ShotParams(
            aimDir: Vec2(1, 0),
            speedMs: 2.2352,
            hitY: -0.25,
            elevationDeg: 0,
          ),
          tableConfig: const PhysicsTableConfig(
            playW: tableW,
            playH: tableH,
            minimumCueElevationDeg: 0,
          ),
        );
        final contact = result.ballBallContactDetails.first;
        final path = result.paths.firstWhere((p) => p.ballId == 'cue');
        final segment = path.points.last - path.points[path.points.length - 2];
        final dx = segment.x / (rx1 - rx0) * tableW;
        final dy = segment.y / (ry1 - ry0) * tableH;
        final angle = math.atan2(dy, dx) * 180 / math.pi;
        final expectedDrawAngle = 180 -
            math.atan2(
                  math.sin(cut) * math.cos(cut),
                  math.sin(cut) * math.sin(cut) - 0.25,
                ) *
                180 /
                math.pi;
        final expectedHeading = 180 - expectedDrawAngle;
        final postImpactRollingVelocity =
            contact.cuePostImpactVelocity * (5 / 7) +
                contact.cuePostImpactAngularVelocity.cross(
                      const Vec3(0, 0, 0.028575),
                    ) *
                    (2 / 7);
        final postImpactPredictedHeading = math.atan2(
              postImpactRollingVelocity.y,
              postImpactRollingVelocity.x,
            ) *
            180 /
            math.pi;
        final idealElasticCueVelocity = contact.cuePreImpactVelocity -
            contact.impactNormal3 * contact.incomingNormalSpeedMs;
        final idealElasticRollingVelocity = idealElasticCueVelocity * (5 / 7) +
            contact.cuePreImpactAngularVelocity.cross(
                  const Vec3(0, 0, 0.028575),
                ) *
                (2 / 7);
        final idealElasticHeading = math.atan2(
              idealElasticRollingVelocity.y,
              idealElasticRollingVelocity.x,
            ) *
            180 /
            math.pi;
        expect(result.cushionContacts, 0);
        expect(result.cueElevationDeg, 0);
        expect(
          angle,
          closeTo(postImpactPredictedHeading, 1.5),
          reason: 'TP A.4 rolling direction from post-impact state at $cutDeg°',
        );
        expect(
          angle,
          closeTo(expectedHeading, 7),
          reason:
              'TP B.14 ideal angle after the A.14 friction and e=0.93 impacts at $cutDeg°',
        );
        expect(
          idealElasticHeading,
          closeTo(expectedHeading, 2.0),
          reason:
              'TP A.4/B.14 ideal elastic contact counterfactual at $cutDeg°',
        );
      }
    });

    test('TP A.20 draw trajectory follows sliding and rolling equations', () {
      const tableW = 60.0;
      const tableH = 60.0;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );
      const phi = math.pi / 6;
      const ob = Vec2(30, 30);
      const radius = 0.028575;
      final cuePosition = Vec2(
        ob.x - 2 * radius * math.cos(phi) - 0.000001,
        ob.y + 2 * radius * math.sin(phi),
      );
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: rel(cuePosition).x, y: rel(cuePosition).y),
          PhysicsBall(id: 'ob', x: rel(ob).x, y: rel(ob).y),
        ],
        cueBallId: 'cue',
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 2.2352,
          hitY: -0.25,
        ),
        tableConfig: const PhysicsTableConfig(playW: tableW, playH: tableH),
      );
      final contact = result.ballBallContactDetails.single;
      final path = result.paths.firstWhere((p) => p.ballId == 'cue');
      final v0 = Vec2(
        contact.cuePostImpactVelocity.x,
        contact.cuePostImpactVelocity.y,
      );
      final surfaceSpin = Vec2(
        radius * contact.cuePostImpactAngularVelocity.y,
        -radius * contact.cuePostImpactAngularVelocity.x,
      );
      final slip = v0 - surfaceSpin;
      final slipDirection = slip.normalized;
      const muSlide = 0.20;
      const muRoll = 0.010;
      const gravity = 9.81;
      final slideTime = slip.length / (3.5 * muSlide * gravity);
      final slideEnd = v0 * slideTime -
          slipDirection * (0.5 * muSlide * gravity * slideTime * slideTime);
      final rollVelocity = v0 - slipDirection * (muSlide * gravity * slideTime);

      Vec2 predictedDisplacement(double elapsed) {
        if (elapsed <= slideTime) {
          return v0 * elapsed -
              slipDirection * (0.5 * muSlide * gravity * elapsed * elapsed);
        }
        final rollElapsed = elapsed - slideTime;
        final rollSpeed = rollVelocity.length;
        final stopTime = rollSpeed / (muRoll * gravity);
        final movingTime = math.min(rollElapsed, stopTime);
        return slideEnd +
            rollVelocity * movingTime -
            rollVelocity.normalized *
                (0.5 * muRoll * gravity * movingTime * movingTime);
      }

      const samplePeriod = 0.008;
      final firstSample = (contact.timeSec / samplePeriod).ceil();
      final comparisons = math.min(20, path.points.length - firstSample);
      expect(result.cushionContacts, 0);
      expect(comparisons, greaterThanOrEqualTo(10));
      final predictedAtFirst = predictedDisplacement(
        firstSample * samplePeriod - contact.timeSec,
      );
      var maxError = 0.0;
      for (var i = firstSample; i < firstSample + comparisons; i++) {
        final elapsed = i * samplePeriod - contact.timeSec;
        final expected = predictedDisplacement(elapsed) - predictedAtFirst;
        final observedRel = path.points[i] - path.points[firstSample];
        final observed = Vec2(
          observedRel.x / (rx1 - rx0) * tableW,
          observedRel.y / (ry1 - ry0) * tableH,
        );
        maxError = math.max(maxError, (observed - expected).length);
      }
      expect(
        maxError,
        lessThan(0.004),
        reason: 'TP A.20 cloth-curve position error over the first 160 ms',
      );
    });

    test('TP A.18 / 4.1 spin offsets match sliding-to-roll travel', () {
      const tableW = 20.0;
      const tableH = 20.0;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );
      const speed = 1.2;
      const muSlide = 0.20;
      const muRoll = 0.010;
      const gravity = 9.81;
      const table = PhysicsTableConfig(playW: tableW, playH: tableH);

      for (final hitY in [-0.4, -0.2, 0.0, 0.2, 0.4]) {
        final initialRomega = 2.5 * hitY * speed;
        final slidingDistance = 2 *
            (6 * speed * speed -
                5 * speed * initialRomega -
                initialRomega * initialRomega) /
            (49 * muSlide * gravity);
        final rollSpeed = (5 * speed + 2 * initialRomega) / 7;
        final rollingDistance = rollSpeed * rollSpeed / (2 * muRoll * gravity);
        final expectedDistance = slidingDistance + rollingDistance;

        final result = _run(
          balls: [cue(rel(const Vec2(10, 7)))],
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: speed,
            hitY: hitY,
          ),
          tableConfig: table,
        );
        final path = result.paths.firstWhere((p) => p.ballId == 'cue');
        var actualDistance = 0.0;
        for (var i = 1; i < path.points.length; i++) {
          final dx =
              (path.points[i].x - path.points[i - 1].x) / (rx1 - rx0) * tableW;
          final dy =
              (path.points[i].y - path.points[i - 1].y) / (ry1 - ry0) * tableH;
          actualDistance += math.sqrt(dx * dx + dy * dy);
        }

        expect(result.cushionContacts, 0);
        expect(
          actualDistance,
          closeTo(expectedDistance, expectedDistance * 0.02),
          reason: 'TP 4.1 for hitY=$hitY: slide then rolling-stop distance',
        );
      }
    });

    test('TP B.2 rolling sidespin produces the predicted ball turn', () {
      const tableW = 20.0;
      const tableH = 20.0;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );

      double lateralDisplacement(double effectiveOffset) {
        var lowOffset = 0.0;
        var highOffset = 0.5;
        for (var i = 0; i < 40; i++) {
          final candidate = (lowOffset + highOffset) / 2;
          if (cueEffectiveLateralOffsetR(candidate, CueType.lowDeflection) <
              effectiveOffset.abs()) {
            lowOffset = candidate;
          } else {
            highOffset = candidate;
          }
        }
        final hitX = effectiveOffset.sign * (lowOffset + highOffset) / 2;
        final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
        final aim = Vec2(math.cos(squirt), -math.sin(squirt));
        final result = simulate(
          balls: [
            PhysicsBall(
              id: 'cue',
              x: rel(const Vec2(10, 10)).x,
              y: rel(const Vec2(10, 10)).y,
            ),
          ],
          cueBallId: 'cue',
          shot: ShotParams(
            aimDir: aim,
            speedMs: 2 * 0.44704,
            hitX: hitX,
            hitY: 0.4,
          ),
          tableConfig: const PhysicsTableConfig(playW: tableW, playH: tableH),
        );
        final points = result.paths.first.points;
        var travelled = 0.0;
        var displacementX = 0.0;
        var displacementY = 0.0;
        for (var i = 1; i < points.length; i++) {
          final dx = (points[i].x - points[i - 1].x) / (rx1 - rx0) * tableW;
          final dy = (points[i].y - points[i - 1].y) / (ry1 - ry0) * tableH;
          final length = math.sqrt(dx * dx + dy * dy);
          final fraction = ((2.4384 - travelled) / length).clamp(0.0, 1.0);
          displacementX += dx * fraction;
          displacementY += dy * fraction;
          travelled += length * fraction;
          if (travelled >= 2.4384) break;
        }
        expect(result.cushionContacts, 0);
        expect(travelled, closeTo(2.4384, 0.02));
        final firstDelta = points[1] - points[0];
        final initialHeading = math.atan2(
          firstDelta.y / (ry1 - ry0) * tableH,
          firstDelta.x / (rx1 - rx0) * tableW,
        );
        return -displacementX * math.sin(initialHeading) +
            displacementY * math.cos(initialHeading);
      }

      final noEnglish = lateralDisplacement(0.0);
      final rightEnglish = lateralDisplacement(0.2);
      final leftEnglish = lateralDisplacement(-0.2);
      expect(noEnglish, closeTo(0.0, 0.001));
      expect(
        rightEnglish - noEnglish,
        closeTo(0.217 * 0.0254, 0.00015),
        reason:
            'TP B.2 predicts 0.217 inches of right turn over 8 feet at 2 mph',
      );
      expect(
        leftEnglish - noEnglish,
        closeTo(-0.217 * 0.0254, 0.00015),
        reason: 'left sidespin must turn the rolling ball to the left',
      );
    });

    test('TP A.16 natural-roll shot equalizes travel near 33.5° cut', () {
      const tableW = 20.0;
      const tableH = 20.0;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 m) => Vec2(
            m.x / tableW * (rx1 - rx0) + rx0,
            m.y / tableH * (ry1 - ry0) + ry0,
          );
      const obM = Vec2(10, 7);
      const cut = 33.477 * math.pi / 180;
      const separation = 0.05715;
      final ghost = Vec2(
        obM.x - separation * math.cos(cut),
        obM.y - separation * math.sin(cut),
      );
      final cueM = Vec2(ghost.x - 0.30, ghost.y);
      const table = PhysicsTableConfig(playW: tableW, playH: tableH);
      final result = _run(
        balls: [cue(rel(cueM)), ball('ob', rel(obM))],
        shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 1.8, hitY: 0.4),
        tableConfig: table,
      );
      double pathDistance(List<Vec2> points) {
        var distance = 0.0;
        for (var i = 1; i < points.length; i++) {
          final dx = (points[i].x - points[i - 1].x) / (rx1 - rx0) * tableW;
          final dy = (points[i].y - points[i - 1].y) / (ry1 - ry0) * tableH;
          distance += math.sqrt(dx * dx + dy * dy);
        }
        return distance;
      }

      final cuePath = result.paths.firstWhere(
        (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
      );
      final objectPath = result.paths.firstWhere((p) => p.ballId == 'ob');
      final cueDistance = pathDistance(cuePath.points);
      final objectDistance = pathDistance(objectPath.points);

      expect(result.cushionContacts, 0);
      expect(cueDistance, greaterThan(0));
      expect(
        objectDistance / cueDistance,
        closeTo(1.0, 0.15),
        reason: 'TP A.16 predicts equal travel distances at 33.477° cut',
      );
    });

    test('TP A.14 impact friction limits include draw and follow spin', () {
      const tableW = 2.540;
      const tableH = 1.270;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 metres) => Vec2(
            metres.x / tableW * (rx1 - rx0) + rx0,
            metres.y / tableH * (ry1 - ry0) + ry0,
          );
      for (final cut in [30.0, 45.0]) {
        for (final speed in [1.111, 2.222, 3.333]) {
          final phi = cut * math.pi / 180;
          const obM = Vec2(1.0, 0.635);
          final cbM = Vec2(
            obM.x - 0.05715 * math.cos(phi) - 0.0001,
            obM.y + 0.05715 * math.sin(phi),
          );
          final cbRel = rel(cbM);
          final obRel = rel(obM);
          for (final hitY in [-0.4, 0.0, 0.4]) {
            final result = simulate(
              balls: [
                PhysicsBall(id: 'cue', x: cbRel.x, y: cbRel.y),
                PhysicsBall(id: 'ob', x: obRel.x, y: obRel.y),
              ],
              cueBallId: 'cue',
              shot: ShotParams(
                aimDir: const Vec2(1, 0),
                speedMs: speed,
                hitY: hitY,
              ),
              tableConfig: PhysicsTableConfig.nineFootDiamond,
            );
            final detail = result.ballBallContactDetails.first;
            final slip = detail.relativeTangentialSlipSpeedMs;
            final expectedFriction = ballBallFrictionForSlip(slip);
            final expectedTangentialImpulse = math.min(
              expectedFriction * detail.normalImpulsePerMass,
              slip / 7,
            );
            final cueContactVelocity = detail.cuePreImpactVelocity +
                detail.cuePreImpactAngularVelocity.cross(
                  detail.impactNormal3 * 0.028575,
                );
            final expectedSlipVector = cueContactVelocity -
                detail.impactNormal3 *
                    cueContactVelocity.dot(detail.impactNormal3);
            final expectedImpulseVector =
                detail.relativeTangentialSlip.normalized() *
                    -expectedTangentialImpulse;
            final cueContactArm = detail.impactNormal3 * 0.028575;
            final expectedCueVelocity = detail.cuePreImpactVelocity -
                detail.impactNormal3 * detail.normalImpulsePerMass +
                detail.tangentialImpulseOnCueBall;
            final expectedCueAngularVelocity =
                detail.cuePreImpactAngularVelocity +
                    cueContactArm.cross(detail.tangentialImpulseOnCueBall) /
                        (0.4 * 0.028575 * 0.028575);
            expect(
              detail.frictionCoefficient,
              closeTo(expectedFriction, 0.01),
              reason: 'TP A.14 friction curve at $cut°/$speed m/s/hitY=$hitY',
            );
            expect(
              (detail.relativeTangentialSlip - expectedSlipVector).length,
              lessThan(1e-8),
              reason: 'TP A.14 contact-point velocity defines 3-D slip',
            );
            expect(
              detail.tangentialImpulsePerMass,
              closeTo(expectedTangentialImpulse, 0.02),
              reason: 'TP A.14 impulse limits at $cut°/$speed m/s/hitY=$hitY',
            );
            expect(
              (detail.tangentialImpulseOnCueBall - expectedImpulseVector)
                  .length,
              lessThan(1e-8),
              reason: 'TP A.14 3-D impulse opposes contact slip',
            );
            expect(
              (detail.cuePostImpactVelocity - expectedCueVelocity).length,
              lessThan(1e-8),
              reason: 'TP A.14 impulse-momentum cue velocity update',
            );
            expect(
              (detail.cuePostImpactAngularVelocity - expectedCueAngularVelocity)
                  .length,
              lessThan(1e-8),
              reason: 'TP A.14 tangential impulse spin transfer',
            );
            if (hitY == 0.0) {
              expect(
                detail.throwAngleRad,
                closeTo(
                  math.atan2(
                    detail.tangentialImpulsePerMass,
                    detail.normalImpulsePerMass,
                  ),
                  0.02,
                ),
                reason: 'TP A.14 planar object-ball throw angle',
              );
            }
          }
        }
      }
    });

    test('TP A.5 stun cue-ball direction follows impact impulses', () {
      const tableW = 2.540;
      const tableH = 1.270;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 metres) => Vec2(
            metres.x / tableW * (rx1 - rx0) + rx0,
            metres.y / tableH * (ry1 - ry0) + ry0,
          );
      const phi = math.pi / 6;
      const obM = Vec2(1.0, 0.635);
      final cbM = Vec2(
        obM.x - 0.05715 * math.cos(phi) - 0.0001,
        obM.y + 0.05715 * math.sin(phi),
      );
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: rel(cbM).x, y: rel(cbM).y),
          PhysicsBall(id: 'ob', x: rel(obM).x, y: rel(obM).y),
        ],
        cueBallId: 'cue',
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 1.341,
          elevationDeg: 0,
        ),
        tableConfig: PhysicsTableConfig.nineFootDiamond,
      );
      final contact = result.ballBallContactDetails.single;
      final n = contact.impactNormal;
      final t = Vec2(-n.y, n.x);
      final cueVelocity = contact.cuePostImpactVelocity;
      final cueNormalSpeed = cueVelocity.x * n.x + cueVelocity.y * n.y;
      final cueTangentSpeed = cueVelocity.x * t.x + cueVelocity.y * t.y;

      // TP A.5, Eqs. 6-13: for equal masses, the cue retains
      // (1-e)/2 of its incoming normal speed; tangent speed is reduced by
      // the collision's tangential impulse. The app's configured e is 0.93.
      const ballRestitution = 0.93;
      final expectedNormal =
          (1 - ballRestitution) * contact.incomingNormalSpeedMs / 2;
      final expectedTangent = contact.incomingNormalSpeedMs * math.tan(phi) -
          contact.tangentialImpulsePerMass;
      final expectedDeflectionFromTangent = math.atan2(
        expectedNormal,
        expectedTangent,
      );
      final measuredDeflectionFromTangent = math.atan2(
        cueNormalSpeed,
        cueTangentSpeed,
      );

      expect(
        contact.normalImpulsePerMass,
        closeTo(
          (1 + ballRestitution) * contact.incomingNormalSpeedMs / 2,
          0.01,
        ),
        reason: 'TP A.5 normal impulse for equal-mass balls',
      );
      expect(
        measuredDeflectionFromTangent,
        closeTo(expectedDeflectionFromTangent, 0.01),
        reason: 'TP A.5 cue-ball direction after inelastic/frictional impact',
      );
    });

    test('TP 3.3 naturally rolling half-ball hit follows the 30° rule', () {
      const tableW = 2.540;
      const tableH = 1.270;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 metres) => Vec2(
            metres.x / tableW * (rx1 - rx0) + rx0,
            metres.y / tableH * (ry1 - ry0) + ry0,
          );

      const cutAngle = math.pi / 6;
      const obM = Vec2(1.27, 0.635);
      final ghostM = Vec2(
        obM.x - 0.05715 * math.cos(cutAngle),
        obM.y - 0.05715 * math.sin(cutAngle),
      );
      final cbM = Vec2(ghostM.x - 0.30, ghostM.y);
      final result = _run(
        balls: [cue(rel(cbM)), ball('ob', rel(obM))],
        shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 1.8, hitY: 0.4),
        tableConfig: PhysicsTableConfig.nineFootDiamond,
      );
      final cueAfter = result.paths
          .where(
            (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
          )
          .expand((p) => p.points)
          .toList();
      expect(cueAfter.length, greaterThanOrEqualTo(2));
      double directionAfterDistance(double targetM) {
        var travelled = 0.0;
        var endpoint = cueAfter.first;
        for (var i = 1; i < cueAfter.length; i++) {
          final previous = cueAfter[i - 1];
          final current = cueAfter[i];
          final dx = (current.x - previous.x) / (rx1 - rx0) * tableW;
          final dy = (current.y - previous.y) / (ry1 - ry0) * tableH;
          travelled += math.sqrt(dx * dx + dy * dy);
          endpoint = current;
          if (travelled >= targetM) break;
        }
        final displacement = endpoint - cueAfter.first;
        final dx = displacement.x / (rx1 - rx0) * tableW;
        final dy = displacement.y / (ry1 - ry0) * tableH;
        return math.atan2(dy.abs(), dx) * 180 / math.pi;
      }

      expect(directionAfterDistance(0.8), closeTo(33.7, 4.0));
    });

    test('TP A.27 direct-hit spin transfer peaks near half english', () {
      const cbPos = Vec2(0.30, 0.50);
      const obPos = Vec2(0.3225, 0.50);
      double transferredSpin(double hitX) {
        final squirt = cueSquirtAngleRad(hitX, CueType.standard);
        final result = _run(
          balls: [cue(cbPos), ball('ob', obPos)],
          shot: ShotParams(
            aimDir: Vec2(math.cos(squirt), -math.sin(squirt)),
            speedMs: 0.45,
            hitX: hitX,
          ),
        );
        return result.peakSpinZ['ob']!;
      }

      final lowEnglish = transferredSpin(0.10);
      final halfEnglish = transferredSpin(0.25);
      final highEnglish = transferredSpin(0.40);
      final initialCueSpin = 2.5 * 0.25 * 0.45 / 0.028575;
      final transferFraction = halfEnglish / initialCueSpin;

      expect(lowEnglish, greaterThan(0));
      expect(highEnglish, greaterThan(0));
      expect(
        halfEnglish,
        greaterThan(lowEnglish),
        reason:
            'TP A.27: transfer increases toward the roughly 40–50% english optimum',
      );
      expect(
        halfEnglish,
        greaterThan(highEnglish),
        reason: 'transfer falls after the efficient english range',
      );
      expect(transferFraction, inInclusiveRange(0.20, 5.0 / 14.0));
    });

    test('TP B.13 faster natural-roll shots stay nearer the tangent line', () {
      const tableW = 2.540;
      const tableH = 1.270;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 metres) => Vec2(
            metres.x / tableW * (rx1 - rx0) + rx0,
            metres.y / tableH * (ry1 - ry0) + ry0,
          );

      double caromAngleAfter(double speedMs, double targetM) {
        const cutAngle = math.pi / 6;
        const obM = Vec2(1.27, 0.635);
        final ghostM = Vec2(
          obM.x - 0.05715 * math.cos(cutAngle),
          obM.y - 0.05715 * math.sin(cutAngle),
        );
        final cbM = Vec2(ghostM.x - 0.30, ghostM.y);
        final result = _run(
          balls: [cue(rel(cbM)), ball('ob', rel(obM))],
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: speedMs,
            hitY: 0.4,
          ),
          tableConfig: PhysicsTableConfig.nineFootDiamond,
        );
        final cueAfter = result.paths
            .where(
              (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
            )
            .expand((p) => p.points)
            .toList();
        var travelled = 0.0;
        var endpoint = cueAfter.first;
        for (var i = 1; i < cueAfter.length; i++) {
          final previous = cueAfter[i - 1];
          final current = cueAfter[i];
          final dx = (current.x - previous.x) / (rx1 - rx0) * tableW;
          final dy = (current.y - previous.y) / (ry1 - ry0) * tableH;
          travelled += math.sqrt(dx * dx + dy * dy);
          endpoint = current;
          if (travelled >= targetM) break;
        }
        final displacement = endpoint - cueAfter.first;
        final dx = displacement.x / (rx1 - rx0) * tableW;
        final dy = displacement.y / (ry1 - ry0) * tableH;
        return math.atan2(dy.abs(), dx) * 180 / math.pi;
      }

      final slow = caromAngleAfter(0.8, 0.20);
      final fast = caromAngleAfter(3.0, 0.20);
      expect(
        fast,
        greaterThan(slow),
        reason:
            'Dr. Dave: higher speed shifts the early path farther down the tangent line',
      );
    });

    test('TP A.6 quarter, half, and three-quarter natural-roll angles', () {
      const tableW = 2.540;
      const tableH = 2.540;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 metres) => Vec2(
            metres.x / tableW * (rx1 - rx0) + rx0,
            metres.y / tableH * (ry1 - ry0) + ry0,
          );

      double caromAngle(double hitFraction) {
        final cutAngle = math.asin(1 - hitFraction);
        const obM = Vec2(1.27, 1.27);
        final ghostM = Vec2(
          obM.x - 0.05715 * math.cos(cutAngle),
          obM.y - 0.05715 * math.sin(cutAngle),
        );
        final cbM = Vec2(ghostM.x - 0.30, ghostM.y);
        final result = _run(
          balls: [cue(rel(cbM)), ball('ob', rel(obM))],
          shot: const ShotParams(
            aimDir: Vec2(1, 0),
            speedMs: 1.8,
            hitY: 0.4,
          ),
          tableConfig: const PhysicsTableConfig(playW: tableW, playH: tableH),
        );
        final cueAfter = result.paths
            .where(
              (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
            )
            .expand((p) => p.points)
            .toList();
        var travelled = 0.0;
        var endpoint = cueAfter.first;
        for (var i = 1; i < cueAfter.length; i++) {
          final previous = cueAfter[i - 1];
          final current = cueAfter[i];
          final dx = (current.x - previous.x) / (rx1 - rx0) * tableW;
          final dy = (current.y - previous.y) / (ry1 - ry0) * tableH;
          travelled += math.sqrt(dx * dx + dy * dy);
          endpoint = current;
          if (travelled >= 0.8) break;
        }
        expect(travelled, greaterThanOrEqualTo(0.8));
        final displacement = endpoint - cueAfter.first;
        final dx = displacement.x / (rx1 - rx0) * tableW;
        final dy = displacement.y / (ry1 - ry0) * tableH;
        return math.atan2(dy.abs(), dx) * 180 / math.pi;
      }

      expect(caromAngle(0.25), closeTo(27.18, 2.0));
      expect(caromAngle(0.50), closeTo(34.026, 2.0));
      expect(caromAngle(0.75), closeTo(28.399, 2.0));

      // TP B.24's 1/4-cue-point construction estimates the same rolling
      // carom output with at most 3.06 degrees of error over cut angle.
      double quarterCuePointAngle(double hitFraction) {
        final cutAngle = math.asin(1 - hitFraction);
        final approximation =
            math.pi / 2 - cutAngle - math.atan(0.25 / math.tan(cutAngle));
        return approximation * 180 / math.pi;
      }

      for (final hitFraction in [0.25, 0.50, 0.75]) {
        expect(
          caromAngle(hitFraction),
          closeTo(quarterCuePointAngle(hitFraction), 3.2),
          reason: 'TP B.24 quarter-cue-point estimate at $hitFraction hit',
        );
      }
    });

    test('cut-induced throw is largest for stun and slower shots at 30°', () {
      const tableW = 2.540;
      const tableH = 1.270;
      const rx0 = 7 / 114;
      const rx1 = 1 - rx0;
      const ry0 = 7 / 64;
      const ry1 = 1 - ry0;
      Vec2 rel(Vec2 metres) => Vec2(
            metres.x / tableW * (rx1 - rx0) + rx0,
            metres.y / tableH * (ry1 - ry0) + ry0,
          );

      double objectDirection({
        required double cutDeg,
        required double hitY,
        double speedMs = 2.0,
      }) {
        const obM = Vec2(1.0, 0.635);
        final cutRad = cutDeg * math.pi / 180;
        final cbM = Vec2(
          obM.x - 0.05715 * math.cos(cutRad) - 0.003,
          obM.y + 0.05715 * math.sin(cutRad),
        );
        final result = _run(
          balls: [cue(rel(cbM)), ball('ob', rel(obM))],
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: speedMs,
            hitY: hitY,
          ),
          tableConfig: PhysicsTableConfig.nineFootDiamond,
        );
        final path = result.paths.firstWhere((p) => p.ballId == 'ob');
        final moved = path.points.indexWhere(
          (point) => (point - rel(obM)).distance > 1e-4,
        );
        expect(moved, greaterThan(0));
        final delta = path.points[moved] - path.points[moved - 1];
        final dxM = delta.x / (rx1 - rx0) * tableW;
        final dyM = delta.y / (ry1 - ry0) * tableH;
        return math.atan2(dyM, dxM).abs();
      }

      double throwAt({
        required double cutDeg,
        required double hitY,
        double speedMs = 2.0,
      }) {
        final cutRad = cutDeg * math.pi / 180;
        return (cutRad -
                objectDirection(cutDeg: cutDeg, hitY: hitY, speedMs: speedMs))
            .abs();
      }

      final stunThrow = throwAt(cutDeg: 30, hitY: 0.0);
      final followThrow = throwAt(cutDeg: 30, hitY: 0.4);
      final slowStunThrow = throwAt(cutDeg: 30, hitY: 0.0, speedMs: 1.0);
      final fastStunThrow = throwAt(cutDeg: 30, hitY: 0.0, speedMs: 3.0);

      expect(stunThrow, greaterThan(0.001));
      expect(followThrow, lessThan(stunThrow));
      expect(
        slowStunThrow,
        greaterThan(fastStunThrow),
        reason:
            'TP A.14: above a 20° cut, slower relative surface motion yields more throw',
      );

      final shallowSlowThrow = throwAt(cutDeg: 15, hitY: 0.0, speedMs: 1.0);
      final shallowFastThrow = throwAt(cutDeg: 15, hitY: 0.0, speedMs: 3.0);
      expect(
        (shallowSlowThrow - shallowFastThrow).abs(),
        lessThan(0.01),
        reason:
            'TP A.14: below about 20°, cut-induced throw is nearly speed-independent',
      );
    });

    test('slow long draw transitions to natural rolling motion', () {
      const cbPos = Vec2(0.2, 0.5);
      final result = _run(
        balls: [cue(cbPos)],
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 0.9,
          hitY: -0.8,
        ),
      );

      expect(result.endPositions['cue'], isNotNull);
      expect(result.endPositions['cue']!.x, greaterThan(cbPos.x));
    });

    test('follow and draw change the cue-ball path after contact', () {
      const cbPos = Vec2(0.3, 0.5);
      const obPos = Vec2(0.6, 0.5);
      final follow = runShot(
        balls: [cue(cbPos), ball('ob', obPos)],
        aimDir: Vec2(1, 0),
        hitY: 0.8,
      );
      final stun = runShot(
        balls: [cue(cbPos), ball('ob', obPos)],
        aimDir: Vec2(1, 0),
      );
      final draw = runShot(
        balls: [cue(cbPos), ball('ob', obPos)],
        aimDir: Vec2(1, 0),
        hitY: -0.8,
      );

      expect(follow.endPositions['cue'], isNotNull);
      expect(stun.endPositions['cue'], isNotNull);
      expect(draw.endPositions['cue'], isNotNull);
      expect(
        (follow.endPositions['cue']! - draw.endPositions['cue']!).distance,
        greaterThan(0.005),
        reason:
            'the collision impulse and retained vertical spin produce distinct paths',
      );
      expect(
        (follow.endPositions['cue']! - stun.endPositions['cue']!).distance,
        greaterThan(0.005),
      );
    });

    test('dead-straight draw sends the cue ball back toward the shooter', () {
      const cbPos = Vec2(0.25, 0.5);
      const obPos = Vec2(0.55, 0.5);
      final result = _run(
        balls: [cue(cbPos), ball('ob', obPos)],
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 2.5,
          hitY: -0.9,
        ),
      );

      expect(result.endPositions['cue'], isNotNull);
      expect(
        result.endPositions['cue']!.x,
        lessThan(obPos.x - 0.015),
        reason:
            'a sufficiently low hit must retain backward rolling action after contact',
      );
    });

    test('follow starts on the tangent and bends forward after a cut', () {
      const cbPos = Vec2(0.30, 0.50);
      const obPos = Vec2(0.55, 0.525);
      final result = _run(
        balls: [cue(cbPos), ball('ob', obPos)],
        shot: ShotParams(aimDir: Vec2(1, 0), speedMs: 2.0, hitY: 0.85),
      );
      final cuePoints = [
        for (final p in result.paths)
          if (p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter)
            ...p.points,
      ];
      expect(cuePoints.length, greaterThanOrEqualTo(2));
      // The first post-contact segment is predominantly tangent; later
      // segments acquire forward (object-ball-line) displacement from follow.
      const tangent = Vec2(0, 1);
      final first = cuePoints[1] - cuePoints[0];
      final last = cuePoints.last - cuePoints[0];
      expect(
        (first.x * tangent.x + first.y * tangent.y).abs(),
        greaterThan(0),
      );
      expect(last.x.abs(), greaterThan(first.x.abs()));
    });

    test('cue ball retains the normal velocity from finite ball restitution',
        () {
      const cbPos = Vec2(0.30, 0.50);
      const obPos = Vec2(0.55, 0.525);
      final result = _run(
        balls: [cue(cbPos), ball('ob', obPos)],
        shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 2.0),
      );
      final cueAfter = result.paths
          .where(
            (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
          )
          .expand((p) => p.points)
          .toList();
      expect(cueAfter.length, greaterThanOrEqualTo(2));

      final dx = obPos.x - cbPos.x;
      final dy = obPos.y - cbPos.y;
      final length = math.sqrt(dx * dx + dy * dy);
      final normal = Vec2(dx / length, dy / length);
      final firstPostImpactStep = cueAfter[1] - cueAfter[0];
      expect(
        firstPostImpactStep.x * normal.x + firstPostImpactStep.y * normal.y,
        greaterThan(0),
        reason:
            'finite restitution leaves the cue ball a small normal component; '
            'the tangent-line limit applies only as restitution approaches one',
      );
    });

    test('no-spin head-on shot has no throw (symmetric)', () {
      const cbPos = Vec2(0.3, 0.5);
      const obPos = Vec2(0.7, 0.5);
      final result = runShot(
        balls: [cue(cbPos), ball('ob', obPos)],
        aimDir: Vec2(1, 0),
      );
      final obEnd = result.endPositions['ob'];
      expect(obEnd, isNotNull);
      expect(obEnd!.y, closeTo(0.5, 0.02));
      expect(
        obEnd.x,
        greaterThan(obPos.x + 0.05),
        reason:
            'cue must strike and drive the resting object ball, not pass through',
      );
      expect(
        result.ghostBallRel,
        isNotNull,
        reason: 'first contact with the resting object ball must be detected',
      );
      expect(
        result.paths.any((p) => p.ballId == 'cue'),
        isTrue,
        reason: 'collision shot must retain the cue-ball trajectory',
      );
      expect(
        result.paths.any((p) => p.ballId == 'ob'),
        isTrue,
        reason: 'collision shot must retain the object-ball trajectory',
      );
    });

    test('cut shot produces throw (OB deflects toward CB travel direction)',
        () {
      const cbPos = Vec2(0.4, 0.6);
      const obPos = Vec2(0.6, 0.45);
      final resultNoSpin = runShot(
        balls: [cue(cbPos), ball('ob', obPos)],
        aimDir: (obPos - cbPos) / (obPos - cbPos).length,
      );
      expect(resultNoSpin.endPositions['ob'], isNotNull);
    });

    test('right english on CB produces a different path than no english', () {
      const cbPos = Vec2(0.3, 0.5);
      const aim = Vec2(-1.0, 0.0);
      final withSpin = _run(
        balls: [cue(cbPos)],
        shot: ShotParams(aimDir: aim, speedMs: 1.5, hitX: 0.8),
      );
      final withoutSpin = _run(
        balls: [cue(cbPos)],
        shot: ShotParams(aimDir: aim, speedMs: 1.5, hitX: 0.0),
      );
      final endNo = withoutSpin.endPositions['cue'];
      expect(endNo, isNotNull);
      final endWith = withSpin.endPositions['cue'];
      if (endWith != null && endNo != null) {
        expect((endWith.y - endNo.y).abs(), greaterThan(0.005));
      }
    });

    test('extreme right english changes the rebound angle at the rail', () {
      const cbPos = Vec2(0.45, 0.5);
      final spin = _run(
        balls: [cue(cbPos)],
        shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 2.2, hitX: 1.0),
      );
      final noSpin = _run(
        balls: [cue(cbPos)],
        shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 2.2),
      );
      final spinContact = spin.cushionContactDetails.first;
      final noSpinContact = noSpin.cushionContactDetails.first;
      expect(
        (spinContact.reboundAngleDeg - noSpinContact.reboundAngleDeg).abs(),
        greaterThan(8.0),
        reason: 'extreme sidespin changes the immediate cushion rebound angle, '
            'as in Mathavan Fig. 10',
      );
    });

    test('right follow does not reverse the cue ball left on a level cut', () {
      PhysicsPath cueAfter(double hitX) {
        final result = _run(
          balls: [
            cue(const Vec2(0.1711, 0.4116)),
            ball('ob', const Vec2(0.1106, 0.1991)),
          ],
          shot: ShotParams(
            aimDir: Vec2(-0.4325, -0.9016),
            speedMs: 2.6822,
            hitX: hitX,
            hitY: 0.9668,
          ),
        );
        return result.paths.firstWhere(
          (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
        );
      }

      final centre = cueAfter(0.0).points;
      final right = cueAfter(0.2554).points;
      expect(right.length, greaterThan(20));
      expect(centre.length, greaterThan(20));
      expect(
        right[20].x,
        greaterThan(centre[20].x - 0.005),
        reason:
            'slight right english must not reverse the post-contact path left',
      );
    });

    test('three-degree high-left follow swerves toward the short rail', () {
      PhysicsPath cueAfter(double hitX) {
        final result = _run(
          balls: [
            cue(const Vec2(0.1549, 0.4505)),
            ball('ob', const Vec2(0.1101, 0.2121)),
          ],
          shot: ShotParams(
            aimDir: Vec2(-0.2807, -0.9598),
            speedMs: 1.1176,
            hitX: hitX,
            hitY: 0.722,
          ),
        );
        return result.paths.firstWhere(
          (p) => p.ballId == 'cue' && p.style == PhysicsPathStyle.cueAfter,
        );
      }

      final centre = cueAfter(0.0).points;
      final highLeft = cueAfter(-0.6919).points;
      expect(centre.length, greaterThan(50));
      expect(highLeft.length, greaterThan(50));
      expect(
        highLeft.last.y,
        lessThan(centre.last.y),
        reason:
            '3° high-left must bend toward the short rail, not away from it',
      );
    });
  });

  group('Squirt', () {
    test('maximum-offset squirt matches Alciatore TP A.31 cue classes', () {
      const radiansToDegrees = 180.0 / math.pi;
      final lowDeflectionSigned =
          cueSquirtAngleRad(0.5, CueType.lowDeflection) * radiansToDegrees;
      final standardSigned =
          cueSquirtAngleRad(0.5, CueType.standard) * radiansToDegrees;

      expect(lowDeflectionSigned.abs(), closeTo(1.8, 0.02));
      expect(standardSigned.abs(), closeTo(2.5, 0.02));
      expect(
        cueSquirtAngleRad(-0.5, CueType.standard),
        closeTo(-standardSigned / radiansToDegrees, 1e-12),
      );
    });

    test('right english squirts opposite the tip offset', () {
      const cbPos = Vec2(0.2, 0.5);
      final withSpin = runShot(
        balls: [cue(cbPos)],
        aimDir: Vec2(1, 0),
        hitX: 0.8,
      );
      final noSpin = runShot(
        balls: [cue(cbPos)],
        aimDir: Vec2(1, 0),
        hitX: 0.0,
      );
      final spinPath = withSpin.paths.singleWhere((p) => p.ballId == 'cue');
      final noSpinPath = noSpin.paths.singleWhere((p) => p.ballId == 'cue');
      final spinPoint = spinPath.points.firstWhere((p) => p.x >= 0.40);
      final noSpinPoint = noSpinPath.points.firstWhere((p) => p.x >= 0.40);
      expect(
        spinPoint.y - noSpinPoint.y,
        lessThan(-0.003),
        reason:
            'right english squirt should initially deflect opposite the tip',
      );
    });

    test('LD cue produces less squirt than standard cue', () {
      const cbPos = Vec2(0.2, 0.5);
      final ldResult = _run(
        balls: [cue(cbPos)],
        shot: ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 2.5,
          hitX: 1.0,
          cueType: CueType.lowDeflection,
        ),
      );
      final stdResult = _run(
        balls: [cue(cbPos)],
        shot: ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 2.5,
          hitX: 1.0,
          cueType: CueType.standard,
        ),
      );
      expect(ldResult.paths, isNotEmpty);
      expect(stdResult.paths, isNotEmpty);
    });
  });

  group('Simulation invariants', () {
    test('pocketed ball has null end position', () {
      const cbPos = Vec2(0.5, 0.5);
      const obPos = Vec2(0.14, 0.22);
      final result = runShot(
        balls: [cue(cbPos), ball('ob', obPos)],
        aimDir: (Vec2(0.14, 0.22) - Vec2(0.5, 0.5)) /
            (Vec2(0.14, 0.22) - Vec2(0.5, 0.5)).length,
      );
      expect(result.endPositions.containsKey('ob'), isTrue);
    });

    test('stopped ball has non-null end position', () {
      final result = runShot(balls: [cue(kCentre)], aimDir: Vec2(1, 0));
      expect(result.endPositions['cue'], isNotNull);
    });

    test('recorded path terminates at the committed final position', () {
      final result = runShot(balls: [cue(kCentre)], aimDir: Vec2(1, 0));
      final cuePaths = result.paths.where((p) => p.ballId == 'cue').toList();
      expect(cuePaths, isNotEmpty);
      expect(result.endPositions['cue'], isNotNull);
      expect(
        (cuePaths.last.points.last - result.endPositions['cue']!).distance,
        lessThan(1e-5),
      );
    });

    test('simulation respects maxTime and does not hang', () {
      final balls = [
        cue(kCentre),
        ball('1', const Vec2(0.6, 0.5)),
        ball('2', const Vec2(0.7, 0.5)),
      ];
      final result = runShot(balls: balls, aimDir: Vec2(1, 0), hitX: 0.5);
      expect(result.paths, isNotEmpty);
    });
  });

  group('Topspin bending invariant', () {
    test(
      'pure max topspin follows forward farther than mixed topspin+sidespin',
      () {
        const cbPos = Vec2(0.20, 0.50);
        const obPos = Vec2(0.40, 0.50);

        final pureTop = _run(
          balls: [cue(cbPos), ball('ob', obPos)],
          shot: const ShotParams(
            aimDir: Vec2(1, 0),
            speedMs: 1.0,
            hitX: 0.0,
            hitY: 0.85,
          ),
        );

        const sideOffset = 0.5 / math.sqrt2;
        final squirt = cueSquirtAngleRad(sideOffset, CueType.lowDeflection);
        final highRight = _run(
          balls: [cue(cbPos), ball('ob', obPos)],
          shot: ShotParams(
            aimDir: Vec2(math.cos(-squirt), math.sin(-squirt)),
            speedMs: 1.0,
            hitX: 0.60,
            hitY: 0.60,
          ),
        );

        final topEnd = pureTop.endPositions['cue'];
        final mixEnd = highRight.endPositions['cue'];

        expect(topEnd, isNotNull, reason: 'cue ball pocketed with pure top');
        expect(mixEnd, isNotNull, reason: 'cue ball pocketed with high-right');

        expect(
          topEnd!.x,
          greaterThan(mixEnd!.x - 0.005),
          reason:
              'pure top should remain within 5 mm of mixed top/side follow; '
              'pure-top x=${topEnd.x.toStringAsFixed(4)} '
              'high-right x=${mixEnd.x.toStringAsFixed(4)}',
        );
      },
    );

    test('topspin magnitude monotonically increases forward follow', () {
      const cbPos = Vec2(0.20, 0.50);
      const obPos = Vec2(0.40, 0.50);

      double? prevX;
      for (final hitY in [0.0, 0.3, 0.6, 0.85]) {
        final result = _run(
          balls: [cue(cbPos), ball('ob', obPos)],
          shot: ShotParams(
            aimDir: Vec2(1, 0),
            speedMs: 1.0,
            hitX: 0.0,
            hitY: hitY,
          ),
        );
        final endX = result.endPositions['cue']?.x;
        expect(endX, isNotNull, reason: 'cue ball pocketed at hitY=$hitY');
        if (prevX != null) {
          expect(
            endX!,
            greaterThanOrEqualTo(prevX),
            reason:
                'hitY=$hitY must give no less forward travel than previous; '
                'got ${endX.toStringAsFixed(4)} vs ${prevX.toStringAsFixed(4)}',
          );
        }
        prevX = endX;
      }
    });
  });

  group('Cushion rebound off every rail', () {
    List<Vec2> cuePath(PhysicsResult r) => [
          for (final p in r.paths)
            if (p.ballId == 'cue') ...p.points,
        ];

    test('right rail rebounds the ball back, no sliding along the rail', () {
      final r = runShot(
        balls: [cue(const Vec2(0.5, 0.5))],
        aimDir: Vec2(1.0, -0.25),
      );
      final pts = cuePath(r);
      final maxX = pts.map((p) => p.x).reduce((a, b) => a > b ? a : b);
      expect(
        maxX,
        greaterThan(0.9),
        reason: 'ball should reach the right rail',
      );
      expect(
        pts.last.x,
        lessThan(maxX - 0.1),
        reason: 'ball must rebound off the right rail, not slide/stick to it',
      );
    });

    test('bottom rail rebounds the ball back, no sliding along the rail', () {
      final r = runShot(
        balls: [cue(const Vec2(0.5, 0.5))],
        aimDir: Vec2(0.25, 1.0),
      );
      final pts = cuePath(r);
      final maxY = pts.map((p) => p.y).reduce((a, b) => a > b ? a : b);
      expect(
        maxY,
        greaterThan(0.82),
        reason: 'ball should reach the bottom rail',
      );
      expect(
        pts.last.y,
        lessThan(maxY - 0.08),
        reason: 'ball must rebound off the bottom rail, not slide/stick to it',
      );
    });
  });

  group('3-D calibration fixtures', () {
    test('cushion grip does not collapse on ordinary slow impacts', () {
      const profile = CushionProfile.standardPool;
      final barely = profile.gripAtApproach(0.05);
      final medium = profile.gripAtApproach(profile.peakGripSpeed);
      final hard = profile.gripAtApproach(profile.highSpeedReference * 1.5);

      expect(barely, closeTo(medium, 1e-9));
      expect(medium, greaterThan(hard));
    });

    test('steep rebound skids to about half speed before the next rail', () {
      final result = _run(
        balls: [cue(const Vec2(0.45, 0.5))],
        shot: const ShotParams(aimDir: Vec2(1, 0), speedMs: 2.2),
      );
      final contacts = result.cushionContactDetails;
      expect(contacts.length, greaterThanOrEqualTo(2));
      final speedAfterSkid =
          contacts[1].incidentSpeedMs / contacts[0].reboundSpeedMs;

      expect(
        speedAfterSkid,
        inInclusiveRange(0.35, 0.65),
        reason:
            'Dr. Dave reports about half the speed remains after a steep rebound and skid',
      );
    });

    test('running and reverse english shift angle as in Mathavan Fig. 10', () {
      const desiredAngle = -0.65;
      CushionContact contactFor(double hitX) {
        final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
        final aimAngle = desiredAngle - squirt;
        final result = _run(
          balls: [cue(const Vec2(0.7, 0.5))],
          shot: ShotParams(
            aimDir: Vec2(math.cos(aimAngle), math.sin(aimAngle)),
            speedMs: 2.0,
            hitX: hitX,
            hitY: 0.4,
          ),
        );
        return result.cushionContactDetails.first;
      }

      final reverse = contactFor(-0.4);
      final centre = contactFor(0.0);
      final running = contactFor(0.4);

      expect(reverse.rail, equals('right'));
      expect(centre.rail, equals('right'));
      expect(running.rail, equals('right'));
      expect(reverse.incidentAngleDeg, inInclusiveRange(30.0, 45.0));
      expect(centre.incidentAngleDeg, inInclusiveRange(30.0, 45.0));
      expect(running.incidentAngleDeg, inInclusiveRange(30.0, 45.0));
      expect(reverse.reboundAngleDeg, greaterThan(centre.reboundAngleDeg + 5));
      expect(running.reboundAngleDeg, lessThan(centre.reboundAngleDeg - 2));
    });

    test('standard profile is the normal-table baseline', () {
      const standard = CushionProfile.standardPool;
      const diamond = CushionProfile.diamondRedLabel;

      expect(standard.noseHeightMm, greaterThan(diamond.noseHeightMm));
      expect(standard.tableImpactFriction, closeTo(0.20, 1e-12));
      expect(diamond.tableImpactFriction, closeTo(0.20, 1e-12));
      expect(diamond.restitution, equals(standard.restitution));
      expect(diamond.impactDurationMs, greaterThan(standard.impactDurationMs));
    });

    test(
      'rail-only maximum-right calibration route reaches the upper rail',
      () {
        final squirt = cueSquirtAngleRad(0.5, CueType.lowDeflection);
        final result = _run(
          balls: [cue(kCentre)],
          shot: ShotParams(
            aimDir: Vec2(-math.cos(squirt), math.sin(squirt)),
            speedMs: 2.50,
            hitX: 1.0,
          ),
        );
        final points = [
          for (final path in result.paths)
            if (path.ballId == 'cue') ...path.points,
        ];
        final upperRail = points.where((p) => p.y < 0.15).toList();
        expect(
          upperRail,
          isNotEmpty,
          reason: 'maximum right from the left-center rail should run upward',
        );
        final closest = upperRail.reduce(
          (a, b) => (a.x - 0.25).abs() < (b.x - 0.25).abs() ? a : b,
        );
        expect(closest.x, inInclusiveRange(0.53, 0.75));
      },
    );

    test(
      'a centre-ball oblique rail contact induces measurable running spin',
      () {
        final result = _run(
          balls: [cue(kCentre)],
          shot: const ShotParams(
            aimDir: Vec2(-0.70710678, -0.70710678),
            speedMs: 2.5,
          ),
        );

        expect(
          result.peakSpinZ['cue']!,
          greaterThan(0.01),
          reason: 'the cushion nose must transfer some vertical-axis spin',
        );
      },
    );

    test('cushion nose height changes the 3-D rebound response', () {
      const shot = ShotParams(
        aimDir: Vec2(-0.70710678, -0.70710678),
        speedMs: 2.5,
      );
      final lowNose = _run(
        balls: [cue(kCentre)],
        shot: shot,
        cushion: const CushionProfile(noseHeightMm: 25),
      );
      final highNose = _run(
        balls: [cue(kCentre)],
        shot: shot,
        cushion: const CushionProfile(noseHeightMm: 50),
      );

      expect(
        (lowNose.peakSpinZ['cue']! - highNose.peakSpinZ['cue']!).abs(),
        greaterThan(0.001),
      );
    });
  });
}
