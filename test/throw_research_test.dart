import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

const _radius = 0.028575;
const _tableW = 2.540;
const _tableH = 1.270;
const _longTableW = 30.0;
const _longTableH = 3.0;
const _rx0 = 7 / 114;
const _ry0 = 7 / 64;
const _spanX = 1 - 14 / 114;
const _spanY = 1 - 14 / 64;

Vec2 _toRelative(Vec2 p) =>
    Vec2(p.x / _tableW * _spanX + _rx0, p.y / _tableH * _spanY + _ry0);

Vec2 _toLongTableRelative(Vec2 p) =>
    Vec2(p.x / _longTableW * _spanX + _rx0, p.y / _longTableH * _spanY + _ry0);

double _tpB8TravelAfterImpact({
  required double speedX,
  required double spinY,
  required double slidingFriction,
  required double rollingResistance,
}) {
  const gravity = 9.81;
  final slip = speedX - _radius * spinY;
  final slideTime = slip.abs() / (3.5 * slidingFriction * gravity);
  final slideAcceleration = -slip.sign * slidingFriction * gravity;
  final slideDistance =
      speedX * slideTime + 0.5 * slideAcceleration * slideTime * slideTime;
  final rollSpeed = speedX + slideAcceleration * slideTime;
  final rollDistance = rollSpeed.sign *
      rollSpeed *
      rollSpeed /
      (2 * rollingResistance * gravity);
  return slideDistance + rollDistance;
}

void main() {
  test('TP B.8/B.9 draw state at object contact matches cloth equations', () {
    for (final (speedMph, dragDistanceFt) in [
      (5.0, 0.5),
      (8.0, 1.0),
      (14.0, 1.0),
    ]) {
      const slidingFriction = 0.20;
      const radius = _radius;
      const gravity = 9.81;
      const hitY = -0.4;
      final initialSpeed = speedMph * 0.44704;
      final dragDistance = dragDistanceFt * 0.3048;
      final expectedSpeed = math.sqrt(
        initialSpeed * initialSpeed -
            2 * slidingFriction * gravity * dragDistance,
      );
      final initialSurfaceSpin = 2.5 * hitY * initialSpeed;
      final expectedSurfaceSpin =
          initialSurfaceSpin + 2.5 * (initialSpeed - expectedSpeed);
      final cuePosition = const Vec2(0.5, 0.635);
      final objectPosition = Vec2(
        cuePosition.x + dragDistance + 2 * radius,
        cuePosition.y,
      );
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
          speedMs: initialSpeed,
          hitY: hitY,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: slidingFriction,
          clothRollingResistance: 0,
        ),
        maxTimeSeconds: 1.0,
      );
      final contact = result.ballBallContactDetails.first;
      final actualSurfaceSpin = contact.cuePreImpactAngularVelocity.y * radius;

      expect(
        contact.cuePreImpactVelocity.x,
        closeTo(expectedSpeed, 0.06),
        reason: 'TP B.8 at $speedMph mph over $dragDistanceFt ft',
      );
      expect(
        actualSurfaceSpin,
        closeTo(expectedSurfaceSpin, 0.15),
        reason: 'TP B.8 draw spin at object-ball contact',
      );
      expect(
        -actualSurfaceSpin / contact.cuePreImpactVelocity.x,
        closeTo(-expectedSurfaceSpin / expectedSpeed, 0.04),
        reason: 'TP B.9 draw spin-to-speed ratio at contact',
      );
    }
  });

  test('TP B.9 short-drag spin-to-speed ratio rises with tip offset', () {
    const speedMph = 5.0;
    const dragDistanceFt = 0.5;
    const slidingFriction = 0.20;
    final speedMs = speedMph * 0.44704;
    final dragDistance = dragDistanceFt * 0.3048;
    final ratios = <double>[];

    for (final hitY in [-0.2, -0.35, -0.5]) {
      final cuePosition = const Vec2(0.5, 0.635);
      final objectPosition = Vec2(
        cuePosition.x + dragDistance + 2 * _radius,
        cuePosition.y,
      );
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
          speedMs: speedMs,
          hitY: hitY,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: slidingFriction,
          clothRollingResistance: 0,
        ),
        maxTimeSeconds: 1.0,
      );
      final contact = result.ballBallContactDetails.first;
      ratios.add(
        -contact.cuePreImpactAngularVelocity.y *
            _radius /
            contact.cuePreImpactVelocity.x,
      );
    }

    expect(ratios[1], greaterThan(ratios[0]));
    expect(ratios[2], greaterThan(ratios[1]));
  });

  test('TP B.8 post-impact draw follows slide-to-roll travel equations', () {
    const slidingFriction = 0.20;
    const rollingResistance = 0.010;
    const targetBallFriction = 0.06;
    // Stop just after the cue reaches rest, before a far-rail rebound can
    // send the object ball back into the cue and contaminate its runout.
    for (final (speedMph, dragDistanceFt, maxTimeSeconds) in [
      (5.0, 0.5, 12.0),
      (8.0, 1.0, 12.0),
      (8.0, 4.0, 15.0),
      (14.0, 2.0, 13.45),
    ]) {
      final speedMs = speedMph * 0.44704;
      final dragDistance = dragDistanceFt * 0.3048;
      final cuePosition = const Vec2(15.0, 1.5);
      const gap = 0.005;
      final objectPosition = Vec2(
        cuePosition.x + dragDistance + 2 * _radius + gap,
        cuePosition.y,
      );
      final result = simulate(
        balls: [
          PhysicsBall(
            id: 'cue',
            x: _toLongTableRelative(cuePosition).x,
            y: _toLongTableRelative(cuePosition).y,
          ),
          PhysicsBall(
            id: 'ob',
            x: _toLongTableRelative(objectPosition).x,
            y: _toLongTableRelative(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: speedMs,
          hitY: -0.4,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _longTableW,
          playH: _longTableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: slidingFriction,
          clothRollingResistance: rollingResistance,
        ),
        ballBallFrictionScale:
            targetBallFriction / ballBallFrictionForSlip(speedMs),
        maxTimeSeconds: maxTimeSeconds,
      );
      final contact = result.ballBallContactDetails.first;
      expect(
        result.cushionContactDetails.where(
          (contact) => contact.ballId == 'cue',
        ),
        isEmpty,
      );
      expect(result.ballBallContactDetails, hasLength(1));
      expect(
        result.finalLinearVelocities['cue']!.length,
        lessThan(0.01),
        reason:
            'cue should have reached rest at $speedMph mph / $dragDistanceFt ft',
      );
      final expectedPostImpactTravel = _tpB8TravelAfterImpact(
        speedX: contact.cuePostImpactVelocity.x,
        spinY: contact.cuePostImpactAngularVelocity.y,
        slidingFriction: slidingFriction,
        rollingResistance: rollingResistance,
      );
      final expectedNetTravel = dragDistance + gap + expectedPostImpactTravel;
      final cuePositionRel = _toLongTableRelative(cuePosition);
      final actualNetTravel =
          (result.endPositions['cue']!.x - cuePositionRel.x) /
              _spanX *
              _longTableW;

      expect(
        actualNetTravel,
        closeTo(expectedNetTravel, 0.025),
        reason: 'TP B.8 at $speedMph mph over $dragDistanceFt ft: '
            'expected $expectedNetTravel m, got $actualNetTravel m',
      );
    }
  });

  test('TP B.8 long draw maximum is below the miscue limit', () {
    const cueSpeedMph = 12.5;
    const dragDistanceFt = 8.0;
    const gap = 0.005;
    const fractionsOfMiscueOffset = [0.50, 0.60, 0.70, 0.76, 0.80, 0.90, 1.0];
    final cueSpeedMs = cueSpeedMph * 0.44704;
    final dragDistance = dragDistanceFt * 0.3048;
    final cuePosition = const Vec2(5.0, 1.5);
    final objectPosition = Vec2(
      cuePosition.x + dragDistance + 2 * _radius + gap,
      cuePosition.y,
    );
    final drawDistances = <double>[];
    final contactSpinRates = <double>[];

    for (final fraction in fractionsOfMiscueOffset) {
      final offsetR = 0.5 * fraction;
      final cbSpeed = cueSpeedMs *
          cueBallSpeedRatioFromCueImpact(
            tipOffsetR: offsetR,
            energyEfficiency: 0.87,
          );
      final cuePositionRel = _toLongTableRelative(cuePosition);
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: cuePositionRel.x, y: cuePositionRel.y),
          PhysicsBall(
            id: 'ob',
            x: _toLongTableRelative(objectPosition).x,
            y: _toLongTableRelative(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: cbSpeed,
          hitY: -offsetR,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _longTableW,
          playH: _longTableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0.20,
          clothRollingResistance: 0.010,
        ),
        maxTimeSeconds: 8.0,
      );
      expect(result.ballBallContactDetails, isNotEmpty);
      final contact = result.ballBallContactDetails.first;
      contactSpinRates.add(contact.cuePreImpactAngularVelocity.y.abs());
      final netTravel = (result.endPositions['cue']!.x - cuePositionRel.x) /
          _spanX *
          _longTableW;
      drawDistances.add(dragDistance + gap - netTravel);
    }

    final bestIndex = drawDistances.indexOf(drawDistances.reduce(math.max));
    final bestOffsetFraction = fractionsOfMiscueOffset[bestIndex];
    expect(
      bestOffsetFraction,
      closeTo(0.763, 0.15),
      reason: 'TP B.8 predicts about 77.5% of maximum tip offset for maximum '
          'draw after an 8 ft drag; simulated distances: $drawDistances ft',
    );
    expect(
      contactSpinRates.last,
      lessThan(contactSpinRates[5]),
      reason: 'TP B.9 predicts that at long drag distance, contact spin can '
          'decrease from 90% offset to the miscue limit',
    );
  });

  test('TP B.9 sticky cloth can reduce long-drag spin ratio at miscue limit',
      () {
    const cueSpeedMs = 12.5 * 0.44704;
    const dragDistance = 8.0 * 0.3048;
    const cuePosition = Vec2(5.0, 1.5);
    final objectPosition = Vec2(
      cuePosition.x + dragDistance + 2 * _radius + 0.005,
      cuePosition.y,
    );
    final cuePositionRel = _toLongTableRelative(cuePosition);
    final ratios = <double>[];

    for (final fraction in [0.90, 1.0]) {
      final offsetR = 0.5 * fraction;
      final speedMs = cueSpeedMs *
          cueBallSpeedRatioFromCueImpact(
            tipOffsetR: offsetR,
            energyEfficiency: 0.87,
          );
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: cuePositionRel.x, y: cuePositionRel.y),
          PhysicsBall(
            id: 'ob',
            x: _toLongTableRelative(objectPosition).x,
            y: _toLongTableRelative(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: speedMs,
          hitY: -offsetR,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _longTableW,
          playH: _longTableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0.30,
          clothRollingResistance: 0.010,
        ),
        maxTimeSeconds: 8.0,
      );
      final contact = result.ballBallContactDetails.first;
      ratios.add(
        -contact.cuePreImpactAngularVelocity.y *
            _radius /
            contact.cuePreImpactVelocity.x,
      );
    }

    expect(
      ratios[1],
      lessThan(ratios[0]),
      reason: 'TP B.9 notes sticky cloth can reverse the usual long-drag '
          'spin-to-speed benefit near the miscue limit: $ratios',
    );
  });

  test('TP B.8 six-foot draw contact states follow sliding equations', () {
    const dragDistanceFt = 6.0;
    const dragDistance = dragDistanceFt * 0.3048;
    const gap = 0.0;
    const cases = [
      (cueSpeedMph: 11.65, offsetFraction: 1.0),
      (cueSpeedMph: 11.25, offsetFraction: 0.90),
      (cueSpeedMph: 11.10, offsetFraction: 0.75),
      (cueSpeedMph: 12.42, offsetFraction: 0.50),
    ];
    const cuePosition = Vec2(5.0, 1.5);
    const slidingFriction = 0.20;
    const rollingResistance = 0.010;

    for (final testCase in cases) {
      final offsetR = 0.5 * testCase.offsetFraction;
      final cueSpeedMs = testCase.cueSpeedMph * 0.44704;
      final ballSpeedMs = cueSpeedMs *
          cueBallSpeedRatioFromCueImpact(
            tipOffsetR: offsetR,
            energyEfficiency: 0.87,
          );
      final cuePositionRel = _toLongTableRelative(cuePosition);
      final objectPosition = Vec2(
        cuePosition.x + dragDistance + 2 * _radius + gap,
        cuePosition.y,
      );
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: cuePositionRel.x, y: cuePositionRel.y),
          PhysicsBall(
            id: 'ob',
            x: _toLongTableRelative(objectPosition).x,
            y: _toLongTableRelative(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: ballSpeedMs,
          hitY: -offsetR,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _longTableW,
          playH: _longTableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: slidingFriction,
          clothRollingResistance: rollingResistance,
        ),
        maxTimeSeconds: 2.0,
      );

      expect(result.ballBallContactDetails, hasLength(1));
      expect(
        result.cushionContactDetails.where(
          (contact) => contact.ballId == 'cue',
        ),
        isEmpty,
      );
      final contact = result.ballBallContactDetails.single;
      final initialSurfaceSpin = -2.5 * offsetR * ballSpeedMs;
      final expectedContactSpeed = math.sqrt(
        ballSpeedMs * ballSpeedMs - 2 * slidingFriction * 9.81 * dragDistance,
      );
      final expectedContactSurfaceSpin =
          initialSurfaceSpin + 2.5 * (ballSpeedMs - expectedContactSpeed);
      expect(
        contact.cuePreImpactVelocity.x,
        closeTo(expectedContactSpeed, 0.01),
        reason:
            'TP B.8 contact speed: ${testCase.cueSpeedMph} mph cue stroke at '
            '${(testCase.offsetFraction * 100).round()}% tip offset',
      );
      expect(
        contact.cuePreImpactAngularVelocity.y * _radius,
        closeTo(expectedContactSurfaceSpin, 0.03),
        reason: 'TP B.8 contact draw spin after a six-foot drag',
      );
    }
  });

  test('TP B.5 rolling direct-hit travel ratio matches the reference', () {
    const gap = 0.005;
    for (final speedMph in [3.0, 7.0]) {
      final speedMs = speedMph * 0.44704;
      final cuePosition = const Vec2(4.0, 1.5);
      final objectPosition = Vec2(
        cuePosition.x + 2 * _radius + gap,
        cuePosition.y,
      );
      final cuePositionRel = _toLongTableRelative(cuePosition);
      final objectPositionRel = _toLongTableRelative(objectPosition);
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: cuePositionRel.x, y: cuePositionRel.y),
          PhysicsBall(id: 'ob', x: objectPositionRel.x, y: objectPositionRel.y),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: speedMs,
          hitY: 0.4,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _longTableW,
          playH: _longTableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0.20,
          clothRollingResistance: 0.010,
        ),
        ballBallFrictionScale: 0.06 / ballBallFrictionForSlip(speedMs),
        maxTimeSeconds: 30.0,
      );
      final cueDistance = (result.endPositions['cue']!.x - cuePositionRel.x) /
              _spanX *
              _longTableW -
          gap;
      final objectDistance =
          (result.endPositions['ob']!.x - objectPositionRel.x) /
              _spanX *
              _longTableW;

      expect(
        objectDistance / cueDistance,
        closeTo(6.08, 0.9),
        reason:
            'TP B.5 predicts an OB/CB travel ratio of 6.08 at $speedMph mph',
      );
    }
  });

  test('TP 3.2 stun-shot travel ratio follows squared speed ratio', () {
    const tableW = 30.0;
    const tableH = 30.0;
    const spanX = 1 - 14 / 114;
    const spanY = 1 - 14 / 64;
    Vec2 relative(Vec2 position) => Vec2(
          position.x / tableW * spanX + _rx0,
          position.y / tableH * spanY + _ry0,
        );

    double pathLength(PhysicsPath path) {
      var length = 0.0;
      for (var i = 1; i < path.points.length; i++) {
        final dx = (path.points[i].x - path.points[i - 1].x) / spanX * tableW;
        final dy = (path.points[i].y - path.points[i - 1].y) / spanY * tableH;
        length += math.sqrt(dx * dx + dy * dy);
      }
      return length;
    }

    for (final cutDeg in [15.0, 30.0, 45.0, 60.0]) {
      final cut = cutDeg * math.pi / 180;
      const incomingSpeed = 2.0;
      const approachDistance = 0.30;
      const slidingFriction = 0.20;
      final speedAtContact = math.sqrt(
        incomingSpeed * incomingSpeed -
            2 * slidingFriction * 9.81 * approachDistance,
      );
      // Centre hit would acquire follow during the 0.30 m run-up. This offset
      // cancels that drag so TP 3.2's stun-at-contact premise is preserved.
      final hitY = -(incomingSpeed - speedAtContact) / incomingSpeed;
      final objectPosition = const Vec2(15.0, 15.0);
      final cuePosition = objectPosition -
          Vec2(2 * _radius * math.cos(cut) + 0.30, 2 * _radius * math.sin(cut));
      final result = simulate(
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
          aimDir: Vec2(1, 0),
          speedMs: incomingSpeed,
          hitY: hitY,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: tableW,
          playH: tableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: slidingFriction,
          clothRollingResistance: 0.010,
        ),
        // TP 3.2's ideal stun equations neglect ball-ball tangential friction.
        ballBallFrictionScale: 0,
      );
      final contact = result.ballBallContactDetails.first;
      final cuePath = result.paths.firstWhere(
        (path) =>
            path.ballId == 'cue' && path.style == PhysicsPathStyle.cueAfter,
      );
      final objectPath = result.paths.firstWhere((path) => path.ballId == 'ob');
      final predictedDistanceRatio = math
          .pow(
            contact.normalImpulsePerMass / contact.cuePostImpactVelocity.length,
            2,
          )
          .toDouble();
      final measuredDistanceRatio =
          pathLength(objectPath) / pathLength(cuePath);
      final preImpactStunRatio = contact.cuePreImpactAngularVelocity.y *
          _radius /
          contact.cuePreImpactVelocity.x;

      expect(
        preImpactStunRatio.abs(),
        lessThan(0.01),
        reason: 'TP 3.2 stun premise at $cutDeg°: Rω/v=$preImpactStunRatio',
      );
      expect(
        measuredDistanceRatio / predictedDistanceRatio,
        closeTo(1, 0.02),
        reason: 'TP 3.2 at $cutDeg°: roll-out distance ratio is the squared '
            'post-impact speed ratio; measured=$measuredDistanceRatio '
            'predicted=$predictedDistanceRatio OB=${pathLength(objectPath)} '
            'CB=${pathLength(cuePath)} postCB=${contact.cuePostImpactVelocity}',
      );
    }
  });

  test('TP A.17 friction scaling changes stun and natural-roll throw', () {
    BallBallContact contactAt({
      required double frictionScale,
      required double hitY,
      required double cutDeg,
    }) {
      final cut = cutDeg * math.pi / 180;
      final objectPosition = const Vec2(1.2, 0.635);
      final cuePosition = objectPosition -
          Vec2(
            2 * _radius * math.cos(cut) + 0.005,
            2 * _radius * math.sin(cut),
          );
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
          speedMs: 3 * 0.44704,
          hitY: hitY,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0,
          clothRollingResistance: 0,
        ),
        ballBallFrictionScale: frictionScale,
      );
      expect(result.ballBallContactDetails, isNotEmpty);
      return result.ballBallContactDetails.first;
    }

    for (final (label, hitY) in [('stun', 0.0), ('natural roll', 0.4)]) {
      for (final cutDeg in [30.0, 60.0]) {
        final contacts = [
          for (final scale in [0.5, 1.0, 1.5])
            contactAt(frictionScale: scale, hitY: hitY, cutDeg: cutDeg),
        ];
        final throws = contacts.map((c) => c.throwAngleRad.abs()).toList();
        expect(
          throws[0],
          lessThan(throws[1]),
          reason: '$label at $cutDeg°: half friction reduces throw',
        );
        expect(
          throws[1],
          lessThan(throws[2]),
          reason: '$label at $cutDeg°: 50% more friction increases throw',
        );

        for (var i = 0; i < contacts.length; i++) {
          final scale = [0.5, 1.0, 1.5][i];
          final contact = contacts[i];
          expect(
            contact.frictionCoefficient,
            closeTo(
              ballBallFrictionForSlip(
                    contact.relativeTangentialSlipSpeedMs,
                  ) *
                  scale,
              1e-12,
            ),
          );
        }
      }
    }
  });

  test('TP B.31 maximum-CIT spin switches from inside to outside', () {
    double predictedThrow(double speed, double cutRad, double spinPercent) {
      // TP B.31 models inside spin as omegaZ = -omega(v, PS), so its lateral
      // relative slip is v*sin(phi) + 1.25*v*PS.
      final lateralSlip = speed * math.sin(cutRad) + 1.25 * speed * spinPercent;
      final relativeSlip = lateralSlip.abs();
      if (relativeSlip < 1e-12) return 0;
      final friction = ballBallFrictionForSlip(relativeSlip);
      final impulseRatio = math.min(
        friction * speed * math.cos(cutRad) / relativeSlip,
        1 / 7,
      );
      return math.atan(impulseRatio * lateralSlip / (speed * math.cos(cutRad)));
    }

    double predictedOptimum(double speed, double cutRad) {
      var bestPercent = 0.0;
      var maximumThrow = double.negativeInfinity;
      for (var i = -1000; i <= 1000; i++) {
        final percent = i / 1000;
        final throwAngle = predictedThrow(speed, cutRad, percent);
        if (throwAngle > maximumThrow) {
          maximumThrow = throwAngle;
          bestPercent = percent;
        }
      }
      return bestPercent;
    }

    double simulatedThrow(double speed, double cutRad, double spinPercent) {
      // In this fixture, positive hitX is inside english for the shallow cut.
      final hitX = 0.5 * spinPercent;
      final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
      final aimDir = Vec2(math.cos(-squirt), math.sin(-squirt));
      final obM = const Vec2(1.2, 0.635);
      final cbM = Vec2(
        obM.x - 2 * _radius * math.cos(cutRad) - 0.005,
        obM.y - 2 * _radius * math.sin(cutRad),
      );
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: _toRelative(cbM).x, y: _toRelative(cbM).y),
          PhysicsBall(id: 'ob', x: _toRelative(obM).x, y: _toRelative(obM).y),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: aimDir,
          speedMs: speed,
          hitX: hitX,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
        ),
      );
      expect(result.ballBallContactDetails, isNotEmpty);
      return result.ballBallContactDetails.first.throwAngleRad;
    }

    for (final speed in [1 * 0.44704, 7 * 0.44704]) {
      for (final cutDeg in [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 85.0]) {
        final cutRad = cutDeg * math.pi / 180;
        final expectedPercent = predictedOptimum(speed, cutRad);
        final expectedMaximumThrow = predictedThrow(
          speed,
          cutRad,
          expectedPercent,
        );
        var bestPercent = 0.0;
        var minimumThrow = double.infinity;
        for (var i = -20; i <= 20; i++) {
          final spinPercent = i / 20;
          final throwAngle = simulatedThrow(speed, cutRad, spinPercent);
          if (throwAngle < minimumThrow) {
            minimumThrow = throwAngle;
            bestPercent = spinPercent;
          }
        }
        expect(
          bestPercent,
          closeTo(expectedPercent, 0.075),
          reason: 'TP B.31 optimum at ${speed.toStringAsFixed(3)} m/s, '
              '$cutDeg° cut: expected ${(expectedPercent * 100).toStringAsFixed(1)}% '
              'inside, got ${(bestPercent * 100).toStringAsFixed(1)}%',
        );
        expect(
          -minimumThrow,
          closeTo(expectedMaximumThrow, 0.01),
          reason: 'TP B.31 maximum CIT magnitude at '
              '${speed.toStringAsFixed(3)} m/s, $cutDeg° cut',
        );
      }
    }
  });

  test('TP B.25 slow/fast outside-spin optimum across cut angles', () {
    double predictedThrow(double speed, double cutRad, double spinPercent) {
      final spinSurfaceSpeed = 1.25 * speed * spinPercent;
      final lateralSlip = speed * math.sin(cutRad) - spinSurfaceSpeed;
      final relativeSlip = lateralSlip.abs();
      if (relativeSlip < 1e-12) return 0;
      final friction = ballBallFrictionForSlip(relativeSlip);
      final impulseRatio = math.min(
        friction * speed * math.cos(cutRad) / relativeSlip,
        1 / 7,
      );
      return math.atan(impulseRatio * lateralSlip / (speed * math.cos(cutRad)));
    }

    double predictedOptimum(double speed, double cutRad) {
      var bestPercent = 0.0;
      var bestSit = double.negativeInfinity;
      final noSpinThrow = predictedThrow(speed, cutRad, 0);
      for (var i = 0; i <= 1000; i++) {
        final percent = i / 1000;
        final sit = noSpinThrow - predictedThrow(speed, cutRad, percent);
        if (sit > bestSit) {
          bestSit = sit;
          bestPercent = percent;
        }
      }
      return bestPercent;
    }

    double simulatedThrow(double speed, double cutRad, double spinPercent) {
      final hitX = -0.5 * spinPercent;
      final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
      final aimDir = Vec2(math.cos(-squirt), math.sin(-squirt));
      final obM = const Vec2(1.2, 0.635);
      final cbM = Vec2(
        obM.x - 2 * _radius * math.cos(cutRad) - 0.005,
        obM.y - 2 * _radius * math.sin(cutRad),
      );
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cue', x: _toRelative(cbM).x, y: _toRelative(cbM).y),
          PhysicsBall(id: 'ob', x: _toRelative(obM).x, y: _toRelative(obM).y),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: aimDir,
          speedMs: speed,
          hitX: hitX,
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _tableW,
          playH: _tableH,
          minimumCueElevationDeg: 0,
        ),
      );
      expect(result.ballBallContactDetails, isNotEmpty);
      return result.ballBallContactDetails.first.throwAngleRad;
    }

    for (final speed in [1 * 0.44704, 7 * 0.44704]) {
      for (final cutDeg in [0.0, 15.0, 30.0, 45.0, 60.0, 75.0]) {
        final cutRad = cutDeg * math.pi / 180;
        final expectedPercent = predictedOptimum(speed, cutRad);
        final noSpinThrow = simulatedThrow(speed, cutRad, 0);
        var actualPercent = 0.0;
        var maxSit = double.negativeInfinity;
        for (var i = 0; i <= 20; i++) {
          final percent = i / 20;
          final sit =
              (noSpinThrow - simulatedThrow(speed, cutRad, percent)).abs();
          if (sit > maxSit) {
            maxSit = sit;
            actualPercent = percent;
          }
        }
        expect(
          actualPercent,
          closeTo(expectedPercent, 0.075),
          reason: 'TP B.25 maximum SIT at ${speed.toStringAsFixed(3)} m/s, '
              '$cutDeg° cut: expected ${(expectedPercent * 100).toStringAsFixed(1)}% '
              'outside, got ${(actualPercent * 100).toStringAsFixed(1)}%',
        );
      }
    }
  });
}
