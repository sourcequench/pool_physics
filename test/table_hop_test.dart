import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

const _hopTableW = 6.0;
const _hopTableH = 3.0;
const _hopRailX = 7 / 114;
const _hopRailY = 7 / 64;
const _hopSpanX = 1 - 2 * _hopRailX;
const _hopSpanY = 1 - 2 * _hopRailY;
const _hopRadius = 0.028575;

Vec2 _relativePosition(Vec2 metres) => Vec2(
      metres.x / _hopTableW * _hopSpanX + _hopRailX,
      metres.y / _hopTableH * _hopSpanY + _hopRailY,
    );

void main() {
  test('TP 7.3 airborne rail impact transfers vertical-plane spin', () {
    const tableW = 2.0;
    const tableH = 1.0;
    const railX = 7 / 114;
    const railY = 7 / 64;
    const spanX = 1 - 2 * railX;
    const spanY = 1 - 2 * railY;
    Vec2 relative(Vec2 p) =>
        Vec2(p.x / tableW * spanX + railX, p.y / tableH * spanY + railY);

    const radius = _hopRadius;
    const restitution = 0.70;
    // Keep the rail contact sliding throughout the impulse. TP 7.3's closed
    // form assumes kinetic friction and does not model stick/slip switching.
    const railFriction = 0.05;
    const impactHeightAboveCenter = 0.08 * radius;
    const cueSpeed = 3.0;
    const hitY = 0.40;
    const elevationDeg = 6.0;
    final elevation = elevationDeg * math.pi / 180;
    // The cue's downward launch first rebounds from the cloth with the
    // simulator's table COR before rising to the airborne apex.
    const tableRestitution = 0.60;
    final apexTime = tableRestitution * cueSpeed * math.sin(elevation) / 9.81;
    final railDistance = cueSpeed * math.cos(elevation) * apexTime;
    final cuePosition = Vec2(tableW - radius - railDistance, tableH / 2);
    final result = simulate(
      balls: [
        PhysicsBall(
          id: 'cue',
          x: relative(cuePosition).x,
          y: relative(cuePosition).y,
        ),
      ],
      cueBallId: 'cue',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: cueSpeed,
        hitY: hitY,
        elevationDeg: elevationDeg,
      ),
      tableConfig: const PhysicsTableConfig(
        playW: tableW,
        playH: tableH,
        minimumCueElevationDeg: 0,
      ),
      cushion: CushionProfile(
        noseHeightMm: (radius + impactHeightAboveCenter) * 1000,
        restitution: restitution,
        airborneRestitution: restitution,
        slidingFriction: railFriction,
        tableImpactFriction: 0,
        highSpeedFriction: railFriction,
        peakGripSpeed: 10.0,
        highSpeedReference: 12.0,
      ),
      maxTimeSeconds: 1.0,
    );

    expect(result.cushionContactDetails, isNotEmpty);
    final contact = result.cushionContactDetails.first;
    expect(contact.rail, 'right');
    expect(contact.timeSec, closeTo(apexTime, 0.003));
    final omegaBefore = contact.spinRpmBefore * 2 * math.pi / 60;
    final predictedOmegaAfter = 5 *
            (1 + restitution) /
            (2 * radius * radius) *
            (railFriction * radius + impactHeightAboveCenter) *
            contact.incidentSpeedMs -
        omegaBefore;

    expect(
      contact.spinRpmAfter * 2 * math.pi / 60,
      closeTo(predictedOmegaAfter.abs(), 2.0),
      reason: 'TP 7.3 predicts rail friction and impact height jointly change '
          'rolling spin; expected ${predictedOmegaAfter.toStringAsFixed(2)} rad/s',
    );
  });

  test('TP B.5 rolling cue-ball direct-hit hop matches impulse prediction', () {
    const targetBallFriction = 0.06;
    const ballRestitution = 0.93;
    const gravity = 9.81;
    for (final speedMph in [12.0, 20.0]) {
      final speedMs = speedMph * 0.44704;
      final cuePosition = const Vec2(2.0, 1.5);
      final objectPosition = const Vec2(2.0 + 2 * _hopRadius + 0.005, 1.5);
      final result = simulate(
        balls: [
          PhysicsBall(
            id: 'cue',
            x: _relativePosition(cuePosition).x,
            y: _relativePosition(cuePosition).y,
          ),
          PhysicsBall(
            id: 'ob',
            x: _relativePosition(objectPosition).x,
            y: _relativePosition(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: speedMs,
          hitY: 0.4, // Rω/v = 1: natural roll at impact.
          elevationDeg: 0,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _hopTableW,
          playH: _hopTableH,
          minimumCueElevationDeg: 0,
          clothSlidingFriction: 0,
          clothRollingResistance: 0,
        ),
        // B.5 uses a constant μb=0.06. Scale the empirical A.14 curve so
        // the comparison uses the same stated ball-ball friction assumption.
        ballBallFrictionScale:
            targetBallFriction / ballBallFrictionForSlip(speedMs),
        maxTimeSeconds: 0.20,
      );

      final predictedVerticalSpeed =
          targetBallFriction * (1 + ballRestitution) * speedMs / 2;
      final predictedHopHeight =
          predictedVerticalSpeed * predictedVerticalSpeed / (2 * gravity);
      final actualHopHeight = result.peakHeights['cue']! - _hopRadius;
      final contact = result.ballBallContactDetails.first;
      expect(
        result.ballBallContactDetails,
        isNotEmpty,
        reason: 'the cue must contact the object ball at $speedMph mph',
      );
      expect(
        actualHopHeight,
        // The current 1 ms semi-implicit step slightly underestimates the
        // peak; include that bounded integration error in this impact check.
        closeTo(predictedHopHeight, predictedHopHeight * 0.04 + 2e-5),
        reason: 'TP B.5 at $speedMph mph: expected '
            '${(predictedHopHeight * 1000).toStringAsFixed(2)} mm, got '
            '${(actualHopHeight * 1000).toStringAsFixed(2)} mm; '
            'μ=${contact.frictionCoefficient}, '
            'Jn/m=${contact.normalImpulsePerMass}, '
            'Jt/m=${contact.tangentialImpulsePerMass}',
      );
      final expectedSpinTransfer = 5 *
          targetBallFriction *
          (1 + ballRestitution) *
          speedMs /
          (4 * _hopRadius);
      final cueSpinLoss = contact.cuePreImpactAngularVelocity.y -
          contact.cuePostImpactAngularVelocity.y;
      expect(
        contact.tangentialImpulsePerMass,
        closeTo(targetBallFriction * contact.normalImpulsePerMass, 1e-9),
      );
      expect(cueSpinLoss, closeTo(expectedSpinTransfer, 1e-6));
      expect(
        contact.objectPostImpactAngularVelocity.y.abs(),
        closeTo(expectedSpinTransfer, 1e-6),
        reason: 'TP B.5 predicts equal cue-spin loss and OB-spin gain',
      );
    }
  });

  test('high-elevation massé hops higher as cue elevation increases', () {
    final peakHeights = <double>[];
    for (final elevation in [75.0, 85.0]) {
      final result = simulate(
        balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: 6 * 0.44704,
          hitX: 0.25,
          hitY: -0.25,
          elevationDeg: elevation,
        ),
        tableConfig: const PhysicsTableConfig(playW: 6.0, playH: 3.0),
      );
      expect(result.tableBounceDistancesMeters['cue'], isNotEmpty);
      peakHeights.add(result.peakHeights['cue']! - 0.028575);
    }
    expect(peakHeights[0], greaterThan(0));
    expect(peakHeights[1], greaterThan(peakHeights[0]));
  });

  test('TP A.19 modest-elevation massé direction follows analytic angle', () {
    for (final elevation in [5.0, 10.0, 15.0, 20.0]) {
      final result = simulate(
        balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: 2.0,
          hitX: 0.25,
          hitY: -0.25,
          elevationDeg: elevation,
        ),
        tableConfig: const PhysicsTableConfig(playW: 6.0, playH: 3.0),
      );
      final points = result.paths.first.points;
      const spanX = 1 - 14 / 114;
      const spanY = 1 - 14 / 64;
      var traveled = 0.0;
      var previous = points.first;
      var tangentStart = points.first;
      var tangentEnd = points.first;
      var tangentStarted = false;
      for (final point in points.skip(1)) {
        final dx = (point.x - previous.x) / spanX * 6.0;
        final dy = (point.y - previous.y) / spanY * 3.0;
        traveled += math.sqrt(dx * dx + dy * dy);
        previous = point;
        if (traveled >= 1.0 && !tangentStarted) {
          tangentStart = point;
          tangentStarted = true;
        }
        if (traveled >= 1.2) {
          tangentEnd = point;
          break;
        }
      }
      final dx = (tangentEnd.x - tangentStart.x) / spanX * 6.0;
      final dy = (tangentEnd.y - tangentStart.y) / spanY * 3.0;
      final angle = math.atan2(dy, dx) * 180 / math.pi;
      final phi = elevation * math.pi / 180;
      const radius = 0.028575;
      final predicted = math.atan2(
            0.25 * radius * math.sin(phi),
            radius * math.cos(phi) - 0.25 * radius,
          ) *
          180 /
          math.pi;
      expect(
        (angle - predicted).abs(),
        lessThan(1.5),
        reason: 'TP A.19 predicts ${predicted.toStringAsFixed(2)}° at '
            '$elevation° elevation; allow the slight shortfall noted in the proof',
      );
    }
  });

  test('TP A.19 75-degree cloth-contact curve matches sliding trajectory', () {
    const elevationDeg = 75.0;
    const hitX = 0.25;
    const hitY = -0.25;
    const speed = 6 * 0.44704;
    const sampleTime = 0.400;
    const friction = 0.20;
    const table = PhysicsTableConfig(playW: 6.0, playH: 3.0);
    final phi = elevationDeg * math.pi / 180;
    final squirt = cueSquirtAngleRad(hitX, CueType.lowDeflection);
    final result = simulate(
      balls: [_relativePosition(const Vec2(3.0, 1.5))]
          .map(
            (position) => PhysicsBall(id: 'cue', x: position.x, y: position.y),
          )
          .toList(),
      cueBallId: 'cue',
      shot: ShotParams(
        aimDir: Vec2(math.cos(squirt), -math.sin(squirt)),
        speedMs: speed,
        hitX: hitX,
        hitY: hitY,
        elevationDeg: elevationDeg,
        continuousClothContact: true,
      ),
      tableConfig: table,
      maxTimeSeconds: sampleTime,
    );

    // TP A.19 / A.4: the initial contact-point slip sets a constant Coulomb
    // friction acceleration until rolling begins.
    final slipForward = speed * (math.cos(phi) - 2.5 * hitY);
    final slipLateral = -speed * 2.5 * hitX * math.sin(phi);
    final slipSpeed = math.sqrt(
      slipForward * slipForward + slipLateral * slipLateral,
    );
    final path = result.paths.single.points;
    final start = _relativePosition(const Vec2(3.0, 1.5));
    final slideDuration = slipSpeed / (3.5 * friction * 9.81);

    expect(result.tableBounceCounts['cue'], 0);
    expect(result.peakHeights['cue'], closeTo(_hopRadius, 1e-9));
    expect(slideDuration, greaterThan(sampleTime));
    expect(path.length, 51, reason: 'record every 8 ms through 0.400 s');
    for (var sample = 1; sample < path.length; sample++) {
      final time = sample * 0.008;
      final expectedForward = speed * math.cos(phi) * time -
          friction * 9.81 * slipForward / slipSpeed * time * time / 2;
      final expectedLateral =
          -friction * 9.81 * slipLateral / slipSpeed * time * time / 2;
      final actualForward =
          (path[sample].x - start.x) / _hopSpanX * table.playW;
      final actualLateral =
          (path[sample].y - start.y) / _hopSpanY * table.playH;
      expect(actualForward, closeTo(expectedForward, 0.0005));
      expect(actualLateral, closeTo(expectedLateral, 0.0005));
    }
    final last = path.last;
    expect((last.y - start.y) / _hopSpanY * table.playH, greaterThan(0.08));
  });

  test('cloth friction controls the early curvature of an elevated massé', () {
    double lateralAtTravel(double friction) {
      const table = PhysicsTableConfig(playW: 6, playH: 3);
      final result = simulate(
        balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
        cueBallId: 'cue',
        shot: const ShotParams(
          aimDir: Vec2(1, 0),
          speedMs: 2.5,
          hitX: 0.4,
          elevationDeg: 70,
        ),
        tableConfig: PhysicsTableConfig(
          playW: table.playW,
          playH: table.playH,
          clothSlidingFriction: friction,
        ),
      );
      const spanX = 1 - 14 / 114;
      const spanY = 1 - 14 / 64;
      var traveled = 0.0;
      var previous = result.paths.first.points.first;
      var sample = previous;
      for (final point in result.paths.first.points.skip(1)) {
        final dx = (point.x - previous.x) / spanX * table.playW;
        final dy = (point.y - previous.y) / spanY * table.playH;
        traveled += math.sqrt(dx * dx + dy * dy);
        previous = point;
        sample = point;
        if (traveled >= 0.30) break;
      }
      return ((sample.y - 0.5) / spanY * table.playH).abs();
    }

    final slickCurve = lateralAtTravel(0.12);
    final stickyCurve = lateralAtTravel(0.30);
    expect(stickyCurve, greaterThan(slickCurve * 1.2));
  });

  test('TP B.10 5-degree elevated draw bounce spacing', () {
    final result = simulate(
      balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
      cueBallId: 'cue',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: 11.194 * 0.44704,
        hitY: -0.5,
        elevationDeg: 5,
      ),
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );
    final bounceDistances = result.tableBounceDistancesMeters['cue']!;
    final bounceSpeeds = result.tableBounceSpeedsMs['cue']!;
    final bounceSpins = result.tableBounceSpinRps['cue']!;
    // The first recorded landing is the immediate cue/table impulse. TP B.10's
    // 12 mph cue / half-ball backspin case starts its ballistic sequence after
    // that impulse and predicts 5° hop spacings of 0.848, 0.500, 0.297, and
    // 0.177 ft.
    expect(bounceDistances.length, greaterThanOrEqualTo(5));
    const expectedHopSpacingsFt = [0.848, 0.500, 0.297, 0.177];
    for (var i = 0; i < expectedHopSpacingsFt.length; i++) {
      expect(
        bounceDistances[i + 1] - bounceDistances[i],
        closeTo(expectedHopSpacingsFt[i] * 0.3048, 0.01),
        reason: 'hop ${i + 1} range from Dr. Dave TP B.10',
      );
    }

    const expectedBounceSpeedsMph = [10.855, 10.658, 10.542, 10.473, 10.432];
    const expectedBounceSpinsRps = [32.897, 31.731, 31.031, 30.612, 30.360];
    for (var i = 0; i < expectedBounceSpeedsMph.length; i++) {
      expect(
        bounceSpeeds[i],
        closeTo(expectedBounceSpeedsMph[i] * 0.44704, 0.02),
        reason: 'post-bounce speed ${i + 1} from Dr. Dave TP B.10',
      );
      expect(
        bounceSpins[i],
        closeTo(expectedBounceSpinsRps[i], 0.2),
        reason: 'post-bounce spin ${i + 1} from Dr. Dave TP B.10',
      );
    }
  });

  test('TP B.10 12-degree elevated draw hop data', () {
    final result = simulate(
      balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
      cueBallId: 'cue',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: 11.194 * 0.44704,
        hitY: -0.5,
        elevationDeg: 12,
      ),
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );
    final bounceDistances = result.tableBounceDistancesMeters['cue']!;
    final bounceSpeeds = result.tableBounceSpeedsMs['cue']!;
    final bounceSpins = result.tableBounceSpinRps['cue']!;
    expect(bounceDistances.length, greaterThanOrEqualTo(3));

    const expectedHopSpacingsFt = [1.906, 1.093];
    const expectedBounceSpeedsMph = [10.3, 9.794];
    const expectedBounceSpinsRps = [30.205, 27.423];
    for (var i = 0; i < expectedHopSpacingsFt.length; i++) {
      expect(
        bounceDistances[i + 1] - bounceDistances[i],
        closeTo(expectedHopSpacingsFt[i] * 0.3048, 0.01),
        reason: '12° hop ${i + 1} range from Dr. Dave TP B.10',
      );
      expect(
        bounceSpeeds[i],
        closeTo(expectedBounceSpeedsMph[i] * 0.44704, 0.02),
        reason: '12° post-bounce speed ${i + 1} from Dr. Dave TP B.10',
      );
      expect(
        bounceSpins[i],
        closeTo(expectedBounceSpinsRps[i], 0.2),
        reason: '12° post-bounce spin ${i + 1} from Dr. Dave TP B.10',
      );
    }
  });

  test('TP B.10 elevated draw contact data across bounce-count changes', () {
    // Dr. Dave tabulates 2 ft draw shots at 9°, 10°, 15°, and 16°. The
    // bounce sequence changes discretely between 10° and 15°, producing
    // non-monotonic contact spin; this is a useful check that elevation is
    // modeled as launch + flight/landing physics, not a spin multiplier.
    const cueStart = Vec2(0.35, 0.635);
    const contactTravel = 2 * 0.3048;
    const cases = [
      (elevationDeg: 9.0, spinRps: 29.259, spinRatio: 1.157),
      (elevationDeg: 10.0, spinRps: 28.645, spinRatio: 1.147),
      (elevationDeg: 15.0, spinRps: 29.069, spinRatio: 1.181),
      (elevationDeg: 16.0, spinRps: 28.694, spinRatio: 1.179),
    ];

    for (final testCase in cases) {
      final objectPosition = Vec2(
        cueStart.x + contactTravel + 2 * _hopRadius,
        cueStart.y,
      );
      final result = simulate(
        balls: [
          PhysicsBall(
            id: 'cue',
            x: _relativePosition(cueStart).x,
            y: _relativePosition(cueStart).y,
          ),
          PhysicsBall(
            id: 'ob',
            x: _relativePosition(objectPosition).x,
            y: _relativePosition(objectPosition).y,
          ),
        ],
        cueBallId: 'cue',
        shot: ShotParams(
          aimDir: const Vec2(1, 0),
          speedMs: 11.194 * 0.44704,
          hitY: -0.5,
          elevationDeg: testCase.elevationDeg,
        ),
        tableConfig: const PhysicsTableConfig(
          playW: _hopTableW,
          playH: _hopTableH,
        ),
        maxTimeSeconds: 1.0,
      );
      expect(result.ballBallContactDetails, hasLength(1));
      final contact = result.ballBallContactDetails.single;
      final forwardSpeed = contact.cuePreImpactVelocity.x;
      final backspinRadPerSec = contact.cuePreImpactAngularVelocity.y.abs();
      final backspinRps = backspinRadPerSec / (2 * math.pi);
      final spinRatio = _hopRadius * backspinRadPerSec / forwardSpeed;

      expect(
        backspinRps,
        closeTo(testCase.spinRps, 0.25),
        reason: 'TP B.10 contact spin at ${testCase.elevationDeg}°',
      );
      expect(
        spinRatio,
        closeTo(testCase.spinRatio, 0.015),
        reason: 'TP B.10 contact spin ratio at ${testCase.elevationDeg}°',
      );
    }
  });

  test('airborne cue ball keeps its horizontal component until landing', () {
    final result = simulate(
      balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
      cueBallId: 'cue',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: 2.5,
        elevationDeg: 45,
      ),
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );

    // At 136 ms the ball is still airborne after its first impact. Its x
    // displacement should reflect the horizontal component of the launch
    // vector, without continuous cloth drag acting during the hop.
    final xAt136ms = result.frames[17]['cue']!.x;
    expect(xAt136ms - 0.5, greaterThan(0.055));
  });

  test('landing applies table restitution and records successive hops', () {
    final result = simulate(
      balls: const [PhysicsBall(id: 'cue', x: 0.5, y: 0.5)],
      cueBallId: 'cue',
      shot: const ShotParams(
        aimDir: Vec2(1, 0),
        speedMs: 2.5,
        elevationDeg: 45,
      ),
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );

    expect(result.tableBounceCounts['cue'], greaterThan(1));
    expect(result.peakHeights['cue'], greaterThan(0.05));
    expect(result.endHeights['cue'], closeTo(0.028575, 1e-9));
  });
}
