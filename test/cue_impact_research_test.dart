import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

void main() {
  group('Dr. Dave cue-tip impact proofs', () {
    test('TP 2.1 friction boundary matches the recommended offsets', () {
      expect(requiredCueTipFrictionForOffset(0.5), closeTo(0.577, 0.001));
      expect(requiredCueTipFrictionForOffset(0.55), closeTo(0.659, 0.001));
      expect(maximumNoSlipCueTipOffsetR(0.577), closeTo(0.5, 0.001));
      expect(maximumNoSlipCueTipOffsetR(0.659), closeTo(0.55, 0.001));
    });

    test('TP A.22 tip geometry limits draw offset independently of miscue', () {
      // A.22 reports x_max/R = 0.612 for an average 12.5 mm nickel-radius tip
      // at 4° elevation, and 0.416 for a 14 mm flat-radius tip at 2°.
      expect(
        maximumCueDrawOffsetForTableClearanceR(cueElevationDeg: 4),
        closeTo(0.612, 0.001),
      );
      expect(
        maximumCueDrawOffsetForTableClearanceR(
          cueElevationDeg: 2,
          tipDiameterM: 0.014,
          tipCurvatureRadiusM: 0.0254,
        ),
        closeTo(0.416, 0.001),
      );

      expect(
        constrainCueDrawOffsetR(
          requestedOffsetR: -0.5,
          cueElevationDeg: 2,
          maximumMiscueOffsetR: 0.5,
          tipDiameterM: 0.014,
          tipCurvatureRadiusM: 0.0254,
        ),
        closeTo(-0.416, 0.001),
        reason: 'flat large tip must not intersect the cloth on a draw hit',
      );
      expect(
        constrainCueDrawOffsetR(requestedOffsetR: -0.5, cueElevationDeg: 20),
        closeTo(-0.5, 1e-12),
        reason: 'the TP 2.1 miscue limit still governs when clearance permits',
      );
      expect(
        constrainCueDrawOffsetR(
          requestedOffsetR: 0.5,
          cueElevationDeg: 2,
          tipDiameterM: 0.014,
          tipCurvatureRadiusM: 0.0254,
        ),
        closeTo(0.5, 1e-12),
        reason: 'draw-side cloth clearance must not reduce follow offset',
      );
    });

    test('TP A.30 center-hit COR maps to its collision efficiency', () {
      expect(
        cueImpactEfficiencyFromRestitution(restitution: 0.73),
        closeTo(0.888, 0.001),
      );
    });

    test('TP A.30 offset impact reduces speed while preserving spin ratio', () {
      const massRatio = 6 / 19;
      final centreSpeedRatio = cueBallSpeedRatioFromCueImpact(
        tipOffsetR: 0,
        ballToCueMassRatio: massRatio,
        energyEfficiency: 0.87,
      );
      final offsetSpeedRatio = cueBallSpeedRatioFromCueImpact(
        tipOffsetR: 0.5,
        ballToCueMassRatio: massRatio,
        energyEfficiency: 0.87,
      );
      expect(centreSpeedRatio, greaterThan(offsetSpeedRatio));
      expect(centreSpeedRatio, closeTo(1.275, 0.002));
      expect(offsetSpeedRatio, closeTo(0.746, 0.002));

      // TP A.30's angular impulse (Eq. 6), divided by its linear impulse,
      // agrees with the solid-sphere 5/2 offset spin ratio used by the engine.
      expect(cueBallSpinRatioFromTipOffset(0.5), closeTo(1.25, 1e-12));
    });

    test('TP B.7 low-squirt cue gains the predicted small spin advantage', () {
      const maximumOffset = 0.5;
      final standardOffset = cueEffectiveLateralOffsetR(
        maximumOffset,
        CueType.standard,
      );
      final lowSquirtOffset = cueEffectiveLateralOffsetR(
        maximumOffset,
        CueType.lowDeflection,
      );

      expect(lowSquirtOffset, greaterThan(standardOffset));
      expect(
        lowSquirtOffset / standardOffset - 1,
        closeTo(0.02339, 0.001),
        reason:
            'TP B.7 predicts about 2.34% more spin at maximum offset for LD cues',
      );
    });

    test(
      'TP B.1 measured Players and Predator Z cues reproduce their squirt',
      () {
        const offsetR = 0.51 / 1.125;
        const radiansToDegrees = 180 / math.pi;
        final players =
            cueSquirtAngleRad(offsetR, CueType.playersMeasured).abs() *
                radiansToDegrees;
        final predatorZ =
            cueSquirtAngleRad(offsetR, CueType.predatorZMeasured).abs() *
                radiansToDegrees;

        expect(players, closeTo(2.5, 0.02));
        expect(predatorZ, closeTo(1.8, 0.02));
        expect(players, greaterThan(predatorZ));
      },
    );
  });
}
