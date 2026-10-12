import 'dart:convert';

import 'sim.dart';
import 'types.dart';
import 'vec2.dart';

PhysicsTableConfig _tableFromName(String name) {
  switch (name) {
    case 'eight':
      return PhysicsTableConfig.eightFootDiamond;
    case 'seven':
      return PhysicsTableConfig.sevenFootDiamond;
    default:
      return PhysicsTableConfig.nineFootDiamond;
  }
}

/// Machine-readable validation case for a billiards shot.
///
/// The [reference] trajectory is optional: analytical cases may specify only
/// invariants, while video-derived cases can provide timestamped measurements.
class PhysicsFixture {
  final String id;
  final String source;
  final PhysicsTableConfig tableConfig;
  final List<FixtureBall> balls;
  final FixtureShot shot;
  final FixtureExpectation expectation;

  const PhysicsFixture({
    required this.id,
    required this.source,
    required this.tableConfig,
    required this.balls,
    required this.shot,
    required this.expectation,
  });

  factory PhysicsFixture.fromJson(Map<String, dynamic> json) {
    final table = json['table'] as String? ?? 'nine';
    return PhysicsFixture(
      id: json['id'] as String,
      source: json['source'] as String? ?? 'unattributed',
      tableConfig: _tableFromName(table),
      balls: (json['balls'] as List)
          .map((value) => FixtureBall.fromJson(value as Map<String, dynamic>))
          .toList(),
      shot: FixtureShot.fromJson(json['shot'] as Map<String, dynamic>),
      expectation: FixtureExpectation.fromJson(
        json['expectation'] as Map<String, dynamic>? ?? const {},
      ),
    );
  }

  factory PhysicsFixture.fromJsonString(String source) =>
      PhysicsFixture.fromJson(jsonDecode(source) as Map<String, dynamic>);

  List<PhysicsBall> toPhysicsBalls() => balls
      .map((ball) => PhysicsBall(id: ball.id, x: ball.x, y: ball.y))
      .toList();

  PhysicsResult simulate() => physicsSimulate(
        balls: toPhysicsBalls(),
        cueBallId: shot.cueBallId,
        shot: shot.toShotParams(),
        tableConfig: tableConfig,
      );
}

/// Avoids shadowing the fixture's [PhysicsFixture.simulate] method in callers.
PhysicsResult physicsSimulate({
  required List<PhysicsBall> balls,
  required String cueBallId,
  required ShotParams shot,
  required PhysicsTableConfig tableConfig,
}) =>
    simulate(
      balls: balls,
      cueBallId: cueBallId,
      shot: shot,
      tableConfig: tableConfig,
    );

class FixtureBall {
  final String id;
  final String type;
  final double x;
  final double y;

  const FixtureBall(this.id, this.type, this.x, this.y);

  factory FixtureBall.fromJson(Map<String, dynamic> json) => FixtureBall(
        json['id'] as String,
        json['type'] as String? ?? 'object',
        (json['x'] as num).toDouble(),
        (json['y'] as num).toDouble(),
      );
}

class FixtureShot {
  final String cueBallId;
  final Vec2 aimDir;
  final double speedMs;
  final double hitX;
  final double hitY;
  final double? elevationDeg;

  const FixtureShot({
    required this.cueBallId,
    required this.aimDir,
    required this.speedMs,
    required this.hitX,
    required this.hitY,
    this.elevationDeg,
  });

  factory FixtureShot.fromJson(Map<String, dynamic> json) {
    final aim = (json['aimDir'] as List).cast<num>();
    return FixtureShot(
      cueBallId: json['cueBallId'] as String? ?? 'cue',
      aimDir: Vec2(aim[0].toDouble(), aim[1].toDouble()),
      speedMs: (json['speedMs'] as num).toDouble(),
      hitX: (json['hitX'] as num).toDouble(),
      hitY: (json['hitY'] as num).toDouble(),
      elevationDeg: (json['elevationDeg'] as num?)?.toDouble(),
    );
  }

  ShotParams toShotParams() => ShotParams(
        aimDir: aimDir,
        speedMs: speedMs,
        hitX: hitX,
        hitY: hitY,
        elevationDeg: elevationDeg,
      );
}

class FixtureExpectation {
  final bool? cueMustContactObject;
  final bool? cueMustHavePostContactPath;
  final int? maxCushionContacts;
  final List<TrajectorySample> reference;
  final double positionToleranceM;

  const FixtureExpectation({
    this.cueMustContactObject,
    this.cueMustHavePostContactPath,
    this.maxCushionContacts,
    this.reference = const [],
    this.positionToleranceM = 0.003,
  });

  factory FixtureExpectation.fromJson(Map<String, dynamic> json) =>
      FixtureExpectation(
        cueMustContactObject: json['cueMustContactObject'] as bool?,
        cueMustHavePostContactPath: json['cueMustHavePostContactPath'] as bool?,
        maxCushionContacts: (json['maxCushionContacts'] as num?)?.toInt(),
        reference: (json['reference'] as List? ?? const [])
            .map(
              (value) =>
                  TrajectorySample.fromJson(value as Map<String, dynamic>),
            )
            .toList(),
        positionToleranceM:
            (json['positionToleranceM'] as num? ?? 0.003).toDouble(),
      );
}

class TrajectorySample {
  final double timeSec;
  final String ballId;
  final Vec2 position;

  const TrajectorySample(this.timeSec, this.ballId, this.position);

  factory TrajectorySample.fromJson(Map<String, dynamic> json) {
    final p = (json['position'] as List).cast<num>();
    return TrajectorySample(
      (json['timeSec'] as num).toDouble(),
      json['ballId'] as String,
      Vec2(p[0].toDouble(), p[1].toDouble()),
    );
  }
}

/// One row exported from a measured or digitized trajectory.
///
/// Positions use the same relative table coordinates as [PhysicsPath]. Velocities
/// are optional because many overhead-video trackers provide positions only.
/// Angular velocity requires a marked ball/fiducial and should be omitted when
/// it was not independently measured.
class MeasuredTrajectorySample {
  final String shotId;
  final double timeSec;
  final String ballId;
  final Vec2 position;
  final Vec2? velocity;
  final double? omegaX;
  final double? omegaY;
  final double? omegaZ;
  final String? event;
  final String? rail;
  final double? uncertaintyMm;

  const MeasuredTrajectorySample({
    required this.shotId,
    required this.timeSec,
    required this.ballId,
    required this.position,
    this.velocity,
    this.omegaX,
    this.omegaY,
    this.omegaZ,
    this.event,
    this.rail,
    this.uncertaintyMm,
  });

  factory MeasuredTrajectorySample.fromCsvRow(
    Map<String, String> row, {
    int lineNumber = 0,
  }) {
    String required(String name) {
      final value = row[name]?.trim();
      if (value == null || value.isEmpty) {
        throw FormatException('missing $name at CSV line $lineNumber');
      }
      return value;
    }

    double number(String name) => double.parse(required(name));
    double? optionalNumber(String name) {
      final value = row[name]?.trim();
      return value == null || value.isEmpty ? null : double.parse(value);
    }

    final vx = optionalNumber('vx');
    final vy = optionalNumber('vy');
    if ((vx == null) != (vy == null)) {
      throw FormatException(
        'vx and vy must be provided together at CSV line $lineNumber',
      );
    }

    return MeasuredTrajectorySample(
      shotId: required('shot_id'),
      timeSec: number('time_sec'),
      ballId: required('ball_id'),
      position: Vec2(number('x'), number('y')),
      velocity: vx == null ? null : Vec2(vx, vy!),
      omegaX: optionalNumber('omega_x'),
      omegaY: optionalNumber('omega_y'),
      omegaZ: optionalNumber('omega_z'),
      event: row['event']?.trim().isEmpty ?? true ? null : row['event']!.trim(),
      rail: row['rail']?.trim().isEmpty ?? true ? null : row['rail']!.trim(),
      uncertaintyMm: optionalNumber('uncertainty_mm'),
    );
  }
}

/// Minimal CSV importer for tracker exports.
class MeasuredTrajectory {
  final List<MeasuredTrajectorySample> samples;

  const MeasuredTrajectory(this.samples);

  factory MeasuredTrajectory.fromCsv(String csv) {
    final lines = csv
        .split(RegExp(r'\r?\n'))
        .map((line) => line.trim())
        .where((line) => line.isNotEmpty && !line.startsWith('#'))
        .toList();
    if (lines.isEmpty) {
      throw const FormatException('trajectory CSV is empty');
    }
    final headers =
        lines.first.split(',').map((value) => value.trim()).toList();
    final requiredHeaders = {'shot_id', 'time_sec', 'ball_id', 'x', 'y'};
    final missing = requiredHeaders.difference(headers.toSet());
    if (missing.isNotEmpty) {
      throw FormatException('trajectory CSV missing columns: $missing');
    }

    final samples = <MeasuredTrajectorySample>[];
    for (var index = 1; index < lines.length; index++) {
      final values =
          lines[index].split(',').map((value) => value.trim()).toList();
      if (values.length != headers.length) {
        throw FormatException(
          'CSV line ${index + 1} has ${values.length} fields; expected ${headers.length}',
        );
      }
      samples.add(
        MeasuredTrajectorySample.fromCsvRow(
          Map<String, String>.fromIterables(headers, values),
          lineNumber: index + 1,
        ),
      );
    }
    return MeasuredTrajectory(samples);
  }
}

class FixtureValidationResult {
  final List<String> failures;

  const FixtureValidationResult(this.failures);

  bool get passed => failures.isEmpty;
}

FixtureValidationResult validateFixture(PhysicsFixture fixture) {
  final result = fixture.simulate();
  final failures = <String>[];
  final cuePaths = result.paths.where(
    (path) => path.ballId == fixture.shot.cueBallId,
  );
  final hasCueAfter = cuePaths.any(
    (path) => path.style == PhysicsPathStyle.cueAfter,
  );

  if (fixture.expectation.cueMustContactObject == true &&
      result.ghostBallRel == null) {
    failures.add('cue did not contact an object ball');
  }
  if (fixture.expectation.cueMustHavePostContactPath == true && !hasCueAfter) {
    failures.add('cue has no post-contact trajectory');
  }
  final maxContacts = fixture.expectation.maxCushionContacts;
  if (maxContacts != null && result.cushionContacts > maxContacts) {
    failures.add(
      'cushion contacts ${result.cushionContacts} exceed $maxContacts',
    );
  }

  for (final sample in fixture.expectation.reference) {
    final path = result.paths.where((value) => value.ballId == sample.ballId);
    final points = path.expand((value) => value.points).toList();
    if (points.isEmpty) {
      failures.add('no trajectory for ${sample.ballId}');
      continue;
    }
    final nearest = points.reduce(
      (a, b) => (a - sample.position).distance < (b - sample.position).distance
          ? a
          : b,
    );
    if ((nearest - sample.position).distance >
        fixture.expectation.positionToleranceM) {
      failures.add(
        '${sample.ballId} reference miss ${(nearest - sample.position).distance}',
      );
    }
  }
  return FixtureValidationResult(failures);
}
