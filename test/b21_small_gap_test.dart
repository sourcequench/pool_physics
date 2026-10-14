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
  test('TP B.21 small-gap combo heading matches the throw/geometric model', () {
    // B.21 starts with the first ball already moving; use the cue ball as a
    // proxy for that prescribed incoming state, so no prior CB→OB collision
    // contaminates the comparison. Zero cloth friction isolates the proof's
    // stun, free-flight collision assumptions. Screen y is opposite the
    // physical-coordinate sign used by the proof.
    const gapInches = [0.25, 0.375, 0.5];
    const speedsMph = [1.0, 3.0, 7.0];

    for (final gapIn in gapInches) {
      final gap = gapIn * 0.0254;
      for (final alphaDeg in [5.0, 10.0, 15.0, 20.0]) {
        final alpha = alphaDeg * math.pi / 180;
        final gamma =
            math.asin((2 * _radius + gap) / (2 * _radius) * math.sin(alpha)) -
                alpha;
        final phi = alpha + gamma;

        for (final speedMph in speedsMph) {
          final speedMs = speedMph * 0.44704;
          final firstBallM = const Vec2(1.0, 0.635);
          final secondBallM = Vec2(
            firstBallM.x + 2 * _radius + gap,
            firstBallM.y,
          );
          final result = simulate(
            balls: [
              PhysicsBall(
                id: 'first',
                x: _toRelative(firstBallM).x,
                y: _toRelative(firstBallM).y,
              ),
              PhysicsBall(
                id: 'second',
                x: _toRelative(secondBallM).x,
                y: _toRelative(secondBallM).y,
              ),
            ],
            cueBallId: 'first',
            shot: ShotParams(
              aimDir: Vec2(math.cos(alpha), math.sin(alpha)),
              speedMs: speedMs,
              elevationDeg: 0,
            ),
            tableConfig: const PhysicsTableConfig(
              playW: _tableW,
              playH: _tableH,
              minimumCueElevationDeg: 0,
              clothSlidingFriction: 0,
              clothRollingResistance: 0,
            ),
            // Only the initial collision geometry is under test here. Stop
            // shortly after impact so later pocket/rail events do not add
            // unrelated contacts to the diagnostic list.
            maxTimeSeconds: 0.1,
          );
          final contact = result.ballBallContactDetails.single;

          // B.21's law-of-sines result for the line-of-centers angle and its
          // A.14 stun/no-english throw equation, using the published Marlow
          // friction curve coefficients. The 1/7 cap is the no-slip-reversal
          // limit for two solid spheres.
          final sourceSlipSpeed =
              contact.cuePreImpactVelocity.length * math.sin(phi);
          final friction =
              0.009951 + 0.108 * math.exp(-1.088 * sourceSlipSpeed);
          final throwRad = math.atan(math.min(friction, math.tan(phi) / 7));
          final expectedScreenHeading = -gamma + throwRad;

          final actualNormal = math.atan2(
            contact.impactNormal.y,
            contact.impactNormal.x,
          );
          final actualScreenHeading = actualNormal + contact.throwAngleRad;
          expect(
            actualNormal,
            closeTo(-gamma, 0.05 * math.pi / 180),
            reason: 'B.21 impact geometry: $gapIn" gap, '
                '$alphaDeg° incoming, $speedMph mph',
          );
          expect(
            actualScreenHeading,
            closeTo(expectedScreenHeading, 0.20 * math.pi / 180),
            reason: 'B.21 second-ball heading: $gapIn" gap, '
                '$alphaDeg° incoming, $speedMph mph',
          );

          if (gapIn == 0.375 && speedMph == 3.0 && alphaDeg <= 20) {
            // The proof's 3/8-inch optimum should keep the second-ball line
            // close to straight at a typical medium speed.
            expect(
              actualScreenHeading.abs(),
              lessThan(0.5 * math.pi / 180),
              reason: 'B.21 3/8-inch gap cancellation at $alphaDeg°',
            );
          }
        }
      }
    }
  });
}
