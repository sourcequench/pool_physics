import 'dart:convert';
import 'dart:io';

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

void main() {
  test('measured trajectory CSV preserves optional physical observations', () {
    const csv = '''
shot_id,time_sec,ball_id,x,y,vx,vy,omega_x,omega_y,omega_z,event,rail,uncertainty_mm
rail-01,0.000,cue,0.200,0.500,1.20,0.00,,,12.5,shot,,0.8
rail-01,0.008,cue,0.205,0.500,1.18,0.01,,,,cushion,left,1.0
''';

    final trajectory = MeasuredTrajectory.fromCsv(csv);
    expect(trajectory.samples, hasLength(2));
    expect(trajectory.samples.first.velocity, const Vec2(1.2, 0.0));
    expect(trajectory.samples.first.omegaZ, 12.5);
    expect(trajectory.samples[1].event, 'cushion');
    expect(trajectory.samples[1].rail, 'left');
    expect(trajectory.samples[1].uncertaintyMm, 1.0);
  });

  test('measured trajectory CSV rejects incomplete velocity pairs', () {
    expect(
      () => MeasuredTrajectory.fromCsv(
        'shot_id,time_sec,ball_id,x,y,vx,vy\nshot,0,cue,0.2,0.5,1.0,',
      ),
      throwsFormatException,
    );
  });

  test(
    'machine-readable physics fixtures parse and satisfy declared invariants',
    () {
      final source =
          File('test/data/physics_validation_fixtures.json').readAsStringSync();
      final fixtures = (jsonDecode(source) as List)
          .map(
            (value) => PhysicsFixture.fromJson(value as Map<String, dynamic>),
          )
          .toList();

      expect(fixtures.length, greaterThanOrEqualTo(3));
      for (final fixture in fixtures) {
        final validation = validateFixture(fixture);
        expect(
          validation.passed,
          isTrue,
          reason: 'fixture ${fixture.id}: ${validation.failures.join(", ")}',
        );
      }
    },
  );
}
