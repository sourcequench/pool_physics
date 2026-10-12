import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

const _radius = 0.028575;
const _tableW = 2.540;
const _tableH = 1.270;
const _rx0 = 7 / 114;
const _ry0 = 7 / 64;
const _spanX = 1 - 14 / 114;
const _spanY = 1 - 14 / 64;

Vec2 _toRelative(Vec2 p) =>
    Vec2(p.x / _tableW * _spanX + _rx0, p.y / _tableH * _spanY + _ry0);

void main() {
  test('TP 6.3 bank angle rises according to cushion restitution', () {
    for (final approachAngle in [20.0, 40.0, 55.0]) {
      final angle = approachAngle * math.pi / 180;
      final start = Vec2(1.27, 1.0);
      final result = simulate(
        balls: [
          PhysicsBall(
            id: 'cue',
            x: _toRelative(start).x,
            y: _toRelative(start).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: Vec2(math.cos(angle), math.sin(angle)),
          speedMs: 2.0,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0,
          clothRollingResistance: 0,
        ),
        cushion: const CushionProfile(
          restitution: 0.98,
          slidingFriction: 0,
          tableImpactFriction: 0,
          highSpeedFriction: 0,
          tangentialRestitution: 1,
          highSpeedTangentialRestitution: 1,
        ),
        maxTimeSeconds: 0.50,
      );
      final contact = result.cushionContactDetails.first;
      final expected =
          math.atan(math.tan(contact.incidentAngleDeg * math.pi / 180) / 0.98) *
              180 /
              math.pi;

      expect(contact.rail, 'bottom');
      expect(
        contact.reboundAngleDeg,
        closeTo(expected, 0.3),
        reason: 'TP 6.3 at $approachAngle° incidence with e=0.98',
      );
    }
  });

  test(
    'TP A.15 frozen cue-ball impact shares normal speed with object ball',
    () {
      const objectPosition = Vec2(1.2, 0.635);
      final cuePosition = Vec2(
        objectPosition.x - 2 * _radius,
        objectPosition.y,
      );
      const phi = 30 * math.pi / 180;
      final result = simulate(
        balls: [
          PhysicsBall(
            id: 'cue',
            x: _toRelative(cuePosition).x,
            y: _toRelative(cuePosition).y,
          ),
          PhysicsBall(
            id: 'ob',
            x: _toRelative(objectPosition).x,
            y: _toRelative(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: Vec2(math.cos(phi), math.sin(phi)),
          speedMs: 1.0,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0,
          clothRollingResistance: 0,
        ),
        maxTimeSeconds: 0.004,
        ballBallFrictionScale: 0,
      );

      final contact = result.ballBallContactDetails.first;
      final normal = contact.impactNormal;
      final tangent = Vec2(-normal.y, normal.x);
      final incomingNormal = contact.cuePreImpactVelocity.x * normal.x +
          contact.cuePreImpactVelocity.y * normal.y;
      final incomingTangent = contact.cuePreImpactVelocity.x * tangent.x +
          contact.cuePreImpactVelocity.y * tangent.y;
      final cueNormal = contact.cuePostImpactVelocity.x * normal.x +
          contact.cuePostImpactVelocity.y * normal.y;
      final cueTangent = contact.cuePostImpactVelocity.x * tangent.x +
          contact.cuePostImpactVelocity.y * tangent.y;

      expect(
        cueNormal / incomingNormal,
        closeTo(0.5, 0.02),
        reason: 'TP A.15 predicts the frozen pair share the normal speed',
      );
      final objectVelocity = result.finalLinearVelocities['ob']!;
      final objectNormal =
          objectVelocity.x * normal.x + objectVelocity.y * normal.y;
      expect(objectNormal / incomingNormal, closeTo(0.5, 0.02));
      expect(cueTangent / incomingTangent, closeTo(1, 0.02));
    },
  );

  test('TP A.8 natural-roll English changes the 30-degree carom subtly', () {
    const cutDeg = 30.0;
    final cutRad = cutDeg * math.pi / 180;
    const tableW = 2.540;
    const tableH = 2.540;
    const spanX = 1 - 14 / 114;
    const spanY = 1 - 14 / 64;
    Vec2 relative(Vec2 position) => Vec2(
          position.x / tableW * spanX + _rx0,
          position.y / tableH * spanY + _ry0,
        );
    final objectPosition = const Vec2(1.27, 1.27);

    double finalCueHeading(double hitX) {
      final cuePosition = objectPosition -
          Vec2(
            2 * _radius * math.cos(cutRad) + 0.30,
            2 * _radius * math.sin(cutRad),
          );
      final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
      PhysicsResult run(double frictionScale) => simulate(
            balls: [
              PhysicsBall(
                id: 'cue',
                x: relative(cuePosition).x,
                y: relative(cuePosition).y,
              ),
              PhysicsBall(
                id: 'ob',
                x: relative(objectPosition).x,
                y: relative(objectPosition).y,
              ),
            ],
            cueBallId: 'cue',
            shot: ShotParams(
              aimDir: Vec2(math.cos(-squirt), math.sin(-squirt)),
              speedMs: 2.0,
              hitX: hitX,
              hitY: 0.4,
              elevationDeg: 0,
            ),
            tableConfig: const PhysicsTableConfig(
              playW: tableW,
              playH: tableH,
              minimumCueElevationDeg: 0,
            ),
            ballBallFrictionScale: frictionScale,
          );
      final baseline = run(1);
      expect(baseline.ballBallContactDetails, isNotEmpty);
      final contactSlip =
          baseline.ballBallContactDetails.first.relativeTangentialSlipSpeedMs;
      final result = run(0.06 / ballBallFrictionForSlip(contactSlip));
      expect(result.ballBallContactDetails, isNotEmpty);
      final cuePoints = result.paths
          .where(
            (path) =>
                path.ballId == 'cue' && path.style == PhysicsPathStyle.cueAfter,
          )
          .expand((path) => path.points)
          .toList();
      var traveled = 0.0;
      var endpoint = cuePoints.first;
      for (var i = 1; i < cuePoints.length; i++) {
        final previous = cuePoints[i - 1];
        final current = cuePoints[i];
        final dx = (current.x - previous.x) / spanX * tableW;
        final dy = (current.y - previous.y) / spanY * tableH;
        traveled += math.sqrt(dx * dx + dy * dy);
        endpoint = current;
        if (traveled >= 0.8) break;
      }
      final displacement = endpoint - cuePoints.first;
      final dx = displacement.x / spanX * tableW;
      final dy = displacement.y / spanY * tableH;
      return math.atan2(dy.abs(), dx) * 180 / math.pi;
    }

    final gearingOutside = finalCueHeading(-math.sin(cutRad) / 2.5);
    final inside = finalCueHeading(0.4);

    // TP A.8's e=1, fixed-μ example gives 35.907° for gearing outside English
    // and 34.574° for inside English. The simulator uses e=.93, so check the
    // directional trend rather than force those exact angles.
    expect(inside, lessThan(gearingOutside));
  });

  test(
    'TP 4.2 center-of-percussion offset produces natural roll at impact',
    () {
      final objectPosition = const Vec2(1.2, 0.635);
      final cuePosition = Vec2(
        objectPosition.x - 2 * _radius - 0.001,
        objectPosition.y,
      );

      for (final hitY in [0.3, 0.4, 0.5]) {
        final result = simulate(
          balls: [
            PhysicsBall(
              id: 'cue',
              x: _toRelative(cuePosition).x,
              y: _toRelative(cuePosition).y,
            ),
            PhysicsBall(
              id: 'ob',
              x: _toRelative(objectPosition).x,
              y: _toRelative(objectPosition).y,
            ),
          ],
          cueBallId: 'cue',
          shot: ShotParams(
            aimDir: const Vec2(1, 0),
            speedMs: 2.0,
            hitY: hitY,
            elevationDeg: 0,
          ),
          tableConfig: const PhysicsTableConfig(
            playW: _tableW,
            playH: _tableH,
            minimumCueElevationDeg: 0,
            clothSlidingFriction: 0,
          ),
        );
        final contact = result.ballBallContactDetails.first;
        final measuredSpinRatio = _radius *
            contact.cuePreImpactAngularVelocity.y /
            contact.cuePreImpactVelocity.x;

        expect(
          measuredSpinRatio,
          closeTo(2.5 * hitY, 2e-4),
          reason: 'TP 4.2/A.12 at vertical tip offset $hitY R',
        );
      }
    },
  );

  test('TP A.7/A.26 gearing outside english eliminates throw at contact', () {
    for (final cutDeg in [15.0, 30.0, 45.0, 60.0, 75.0]) {
      final cutRad = cutDeg * math.pi / 180;
      // TP A.7/A.26 gearing condition is v*sin(phi) - R*omegaZ = 0.
      // TP B.7 adjusts the input tip offset for squirt, so invert its effective
      // offset to preserve the intended gearing spin at impact.
      final targetSpinOffset = -math.sin(cutRad) / 2.5;
      var lowOffset = 0.0;
      var highOffset = 0.5;
      for (var i = 0; i < 40; i++) {
        final candidate = (lowOffset + highOffset) / 2;
        if (cueEffectiveLateralOffsetR(candidate, CueType.lowDeflection) <
            targetSpinOffset.abs()) {
          lowOffset = candidate;
        } else {
          highOffset = candidate;
        }
      }
      final hitX = targetSpinOffset.sign * (lowOffset + highOffset) / 2;
      final obM = const Vec2(1.2, 0.635);
      final cbM = Vec2(
        obM.x - 2 * _radius * math.cos(cutRad) - 0.001,
        obM.y - 2 * _radius * math.sin(cutRad),
      );
      final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: _toRelative(cbM).x, y: _toRelative(cbM).y),
          PhysicsBall(id: 'ob', x: _toRelative(obM).x, y: _toRelative(obM).y),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: Vec2(math.cos(-squirt), math.sin(-squirt)),
          speedMs: 3 * 0.44704,
          hitX: hitX,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
        ),
      );
      final contact = result.ballBallContactDetails.single;

      expect(
        contact.relativeTangentialSlipSpeedMs,
        lessThan(0.02),
        reason: 'TP A.7 gearing slip at $cutDeg°',
      );
      expect(
        contact.tangentialImpulsePerMass,
        lessThan(0.003),
        reason: 'gearing contact should have essentially no tangential impulse '
            'at $cutDeg°',
      );
      expect(
        contact.throwAngleRad.abs(),
        lessThan(0.5 * math.pi / 180),
        reason: 'gearing outside english should remove ball throw at $cutDeg°',
      );
    }
  });
}
