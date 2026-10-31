// Stochastic analysis of effective pocket opening.
//
// Cut angle: angle between CB→OB direction and OB→pocket direction.
// 0° = straight-in, 75° ≈ very steep cut.
//
// For each (pocket type × cut angle) we fire 150 shots with aim error drawn
// from N(0, σ) and binary-search for σ₅₀ (σ where make rate ≈ 50%).
// Higher σ₅₀ = more forgiving pocket/angle combo.
//
// Run: dart test test/pocket_opening_test.dart --reporter=expanded
import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

const double _ballRadius = 0.028575; // ball radius, metres

Vec2 _mToRel(Vec2 m, double W, double H) {
  const rx0 = 7.0 / 114.0;
  const rx1 = 1.0 - rx0;
  const ry0 = 7.0 / 64.0;
  const ry1 = 1.0 - ry0;
  return Vec2(m.x / W * (rx1 - rx0) + rx0, m.y / H * (ry1 - ry0) + ry0);
}

// ── Single shot ───────────────────────────────────────────────────────────────

bool _runShot({
  required Vec2 cbM,
  required Vec2 obM,
  required Vec2 nominalAim,
  required double perturbRad,
  required double speedMs,
  required double W,
  required double H,
}) {
  final c = math.cos(perturbRad);
  final s = math.sin(perturbRad);
  final aimDir = Vec2(
    nominalAim.x * c - nominalAim.y * s,
    nominalAim.x * s + nominalAim.y * c,
  );

  final result = simulate(
    balls: [
      PhysicsBall(id: 'cb', x: _mToRel(cbM, W, H).x, y: _mToRel(cbM, W, H).y),
      PhysicsBall(id: 'ob', x: _mToRel(obM, W, H).x, y: _mToRel(obM, W, H).y),
    ],
    cueBallId: 'cb',
    shot: ShotParams(aimDir: aimDir, speedMs: speedMs, hitX: 0, hitY: 0),
    tableConfig: PhysicsTableConfig(playW: W, playH: H),
  );

  return result.endPositions['ob'] == null; // null ⇒ pocketed
}

// ── Scenario: cut angle θ between CB→OB and OB→pocket ───────────────────────

// [pocketDir] is the unit vector pointing INTO the pocket from the play area.
// CB is placed [distCbOb] behind OB in the CB→OB direction which is
// pocketDir rotated by [cutDeg].
({Vec2 cbM, Vec2 obM, Vec2 aimDir}) _buildScenario({
  required Vec2 pocketM,
  required Vec2 pocketDir,
  required double cutDeg,
  required double distObM, // OB → pocket distance
  required double distCbM, // CB → OB distance
}) {
  final obM = pocketM - pocketDir * distObM;
  final ghostM = obM - pocketDir * (2 * _ballRadius);

  final rad = cutDeg * math.pi / 180.0;
  final c = math.cos(rad);
  final s = math.sin(rad);
  // Rotate pocketDir by cutDeg to get the CB→OB direction.
  final cbToOb = Vec2(
    pocketDir.x * c - pocketDir.y * s,
    pocketDir.x * s + pocketDir.y * c,
  );
  final cbM = obM - cbToOb * distCbM;

  final aimVec = ghostM - cbM;
  final aimDir = aimVec / aimVec.distance;

  return (cbM: cbM, obM: obM, aimDir: aimDir);
}

// ── Statistics ────────────────────────────────────────────────────────────────

double _makeRate({
  required Vec2 cbM,
  required Vec2 obM,
  required Vec2 aimDir,
  required double sigmaDeg,
  required double speedMs,
  required double W,
  required double H,
  required math.Random rng,
  required int n,
}) {
  final sigmaRad = sigmaDeg * math.pi / 180.0;
  int made = 0;
  for (int i = 0; i < n; i++) {
    final u1 = math.max(rng.nextDouble(), 1e-10);
    final u2 = rng.nextDouble();
    final z = math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2);
    if (_runShot(
      cbM: cbM,
      obM: obM,
      nominalAim: aimDir,
      perturbRad: z * sigmaRad,
      speedMs: speedMs,
      W: W,
      H: H,
    )) {
      made++;
    }
  }
  return made / n;
}

double _sigma50({
  required Vec2 cbM,
  required Vec2 obM,
  required Vec2 aimDir,
  required double speedMs,
  required double W,
  required double H,
  required math.Random rng,
  int n = 150,
}) {
  double lo = 0.05, hi = 6.0;
  if (_makeRate(
        cbM: cbM,
        obM: obM,
        aimDir: aimDir,
        sigmaDeg: lo,
        speedMs: speedMs,
        W: W,
        H: H,
        rng: rng,
        n: n,
      ) <
      0.50) {
    return 0.0;
  }
  if (_makeRate(
        cbM: cbM,
        obM: obM,
        aimDir: aimDir,
        sigmaDeg: hi,
        speedMs: speedMs,
        W: W,
        H: H,
        rng: rng,
        n: n,
      ) >=
      0.50) {
    return hi;
  }
  for (int i = 0; i < 10; i++) {
    final mid = (lo + hi) / 2;
    final r = _makeRate(
      cbM: cbM,
      obM: obM,
      aimDir: aimDir,
      sigmaDeg: mid,
      speedMs: speedMs,
      W: W,
      H: H,
      rng: rng,
      n: n,
    );
    if (r >= 0.50) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return (lo + hi) / 2;
}

/// Half-width (inches) of the contiguous band of target offsets, measured
/// outward from the pocket centre, that are made. Dr. Dave's target width is
/// this contiguous band. A bisection is not used because offsets just past the
/// edge can still drop after a facing contact, which would make the band look
/// wider than it is.
double _contiguousHalfWidthInches(bool Function(double offsetInches) made) {
  const step = 0.05;
  var offset = 0.0;
  while (offset <= 3.5 && made(offset)) {
    offset += step;
  }
  return math.max(0.0, offset - step / 2);
}

/// Top of Dr. Dave's "fast" cue-ball speed band (4-7 mph = 1.9-3.2 m/s).
const _fastShotSpeedMs = 3.2;

// ── Tests ─────────────────────────────────────────────────────────────────────

void main() {
  const W = 2.540; // 9-foot Diamond
  const H = 1.270;
  const speedMs = 1.8; // pocket speed
  const distOb = 0.6; // OB → pocket (m)
  const distCb = 0.5; // CB → OB (m)

  // Cut angles to test.
  const cuts = [0.0, 20.0, 35.0, 50.0, 65.0, 75.0];

  test('Dr Dave TP 3.5/3.7 side-pocket center-angle target width', () {
    bool madeAtOffset(double offsetInches, double speedMs) {
      final offset = offsetInches * 0.0254;
      final obM = Vec2(W / 2 + offset, 0.60);
      final cbM = Vec2(obM.x, obM.y + 0.50 + 2 * _ballRadius);
      return _runShot(
        cbM: cbM,
        obM: obM,
        nominalAim: const Vec2(0, -1),
        perturbRad: 0,
        speedMs: speedMs,
        W: W,
        H: H,
      );
    }

    double simulatedHalfWidth(double speedMs) =>
        _contiguousHalfWidthInches((offset) => madeAtOffset(offset, speedMs));

    // Digitized at the chart's center angle (θ=0): TP 3.5 slow target width
    // ≈3.3 in; TP 3.7 fast target width ≈2.9 in. The simulator's speed is
    // cue-ball launch speed; at a center hit the OB receives nearly all of it.
    // Fast uses 3.2 m/s, the top of Dr. Dave's "fast" band (4-7 mph,
    // 1.9-3.2 m/s; drdavepoolinfo.com/?p=2486). Far faster shots (e.g. 8 m/s)
    // launch the object ball over the rail cap, outside what TP 3.7 charts.
    final slowHalfWidth = simulatedHalfWidth(1.8);
    final fastHalfWidth = simulatedHalfWidth(_fastShotSpeedMs);
    expect(slowHalfWidth, closeTo(3.3 / 2, 0.30));
    expect(fastHalfWidth, closeTo(2.9 / 2, 0.25));
    expect(slowHalfWidth, greaterThan(fastHalfWidth));
  });

  test('Dr Dave TP 3.6/3.8 corner-pocket center-angle target width', () {
    const invSqrt2 = 1 / math.sqrt2;
    const towardPocket = Vec2(-invSqrt2, -invSqrt2);
    const acrossMouth = Vec2(invSqrt2, -invSqrt2);

    bool madeAtOffset(double offsetInches, double speedMs) {
      final target = acrossMouth * (offsetInches * 0.0254);
      final obM = target - towardPocket * 0.60;
      final cbM = obM - towardPocket * (0.50 + 2 * _ballRadius);
      return _runShot(
        cbM: cbM,
        obM: obM,
        nominalAim: towardPocket,
        perturbRad: 0,
        speedMs: speedMs,
        W: W,
        H: H,
      );
    }

    double simulatedHalfWidth(double speedMs) =>
        _contiguousHalfWidthInches((offset) => madeAtOffset(offset, speedMs));

    // Chart center-angle readings are approximate: TP 3.6 slow ≈2.7 in,
    // TP 3.8 fast ≈2.4 in. Target a broad tolerance for graph digitization,
    // then assert the chart's slow-shot advantage.
    final slowHalfWidth = simulatedHalfWidth(1.8);
    final fastHalfWidth = simulatedHalfWidth(_fastShotSpeedMs);
    expect(slowHalfWidth, closeTo(2.7 / 2, 0.15));
    expect(fastHalfWidth, closeTo(2.4 / 2, 0.25));
    expect(slowHalfWidth, greaterThan(fastHalfWidth));
  });

  test(
    'pocket opening stochastic analysis',
    () {
      final rng = math.Random(42);

      // ── Corner TL (0,0) ───────────────────────────────────────────────────────
      // pocketDir points INTO the corner from inside the table = upper-left.
      final cornerDir = Vec2(-1.0 / math.sqrt(2), -1.0 / math.sqrt(2));

      // ── Top side pocket (W/2, 0) ──────────────────────────────────────────────
      // OB directly below pocket, pocketDir straight up = (0, -1).
      // With distOb=0.6 and distCb=0.5: CB max y = 0.6+0.5=1.1 < H=1.27 (always OK).
      final sideDir = Vec2(0, -1.0);

      final rows = <String>[];

      for (final cut in cuts) {
        final corner = _buildScenario(
          pocketM: Vec2.zero,
          pocketDir: cornerDir,
          cutDeg: cut,
          distObM: distOb,
          distCbM: distCb,
        );
        final side = _buildScenario(
          pocketM: Vec2(W / 2, 0),
          pocketDir: sideDir,
          cutDeg: cut,
          distObM: distOb,
          distCbM: distCb,
        );

        // Verify both are on the table before running.
        bool onTable(Vec2 p) =>
            p.x >= 0.05 && p.x <= W - 0.05 && p.y >= 0.05 && p.y <= H - 0.05;

        String run(String label, ({Vec2 cbM, Vec2 obM, Vec2 aimDir}) sc) {
          if (!onTable(sc.cbM) || !onTable(sc.obM)) return '  $label off-table';
          final sigma = _sigma50(
            cbM: sc.cbM,
            obM: sc.obM,
            aimDir: sc.aimDir,
            speedMs: speedMs,
            W: W,
            H: H,
            rng: rng,
          );
          return '  $label  σ₅₀= ${sigma.toStringAsFixed(2).padLeft(5)}°';
        }

        final cutStr = cut.toStringAsFixed(0).padLeft(2);
        rows.add(run('Corner  cut=$cutStr°', corner));
        rows.add(run('Side    cut=$cutStr°', side));
      }

      // ignore: avoid_print
      print('\n══ Effective Pocket Tolerance (σ at 50% make rate) ══════════');
      // ignore: avoid_print
      print(
        '  OB→pocket: ${distOb}m   CB→OB: ${distCb}m   Speed: ${speedMs}m/s',
      );
      // ignore: avoid_print
      print('  Trials per point: 150  (fixed seed, reproducible)');
      // ignore: avoid_print
      print('─' * 56);
      for (final r in rows) {
        // ignore: avoid_print
        print(r);
      }
      // ignore: avoid_print
      print('─' * 56);
      // ignore: avoid_print
      print('  Higher σ₅₀ = more forgiving.  0.00 = misses perfectly.');

      expect(
        rows.every((r) => !r.contains('off-table')),
        isTrue,
        reason: 'All scenarios should be on the table',
      );
    },
    timeout: const Timeout(Duration(minutes: 15)),
    tags: ['benchmark'],
  );

  // ── Jaw rejection regression test ─────────────────────────────────────────
  test('side pocket jaw rejects ball that clips the nose', () {
    const cfg = PhysicsTableConfig.nineFootDiamond;
    final result = simulate(
      balls: const [
        PhysicsBall(id: 'cb', x: 0.4189, y: 0.4382),
        PhysicsBall(id: 'ob', x: 0.4748, y: 0.2649),
      ],
      cueBallId: 'cb',
      shot: const ShotParams(
        aimDir: Vec2(0.4895, -0.872),
        speedMs: 2.6822,
        hitX: 0,
        hitY: 0,
      ),
      tableConfig: cfg,
    );

    final obPocketed = result.endPositions['ob'] == null;
    final obEnd = result.endPositions['ob'];
    // ignore: avoid_print
    print('\nJaw-clip test: pocketed=$obPocketed  endPos=$obEnd');

    expect(
      obPocketed,
      isFalse,
      reason: 'OB clips side pocket jaw — should bounce back, not pocket',
    );
  });

  // ── Corner pocket jaw rattle regression test ──────────────────────────────
  // Ball shot down the long rail toward the TL corner at speed.  It arrives
  // nearly parallel to the rail, clips the near jaw tip, and should rattle
  // rather than sink.  Before corner-jaw physics, any ball within
  // cornerPocketR of the corner was instantly pocketed.
  test('corner pocket jaw rejects ball shot down the long rail at speed', () {
    // In metres on a 9-foot Diamond: W=2.540, H=1.270.
    // TL-corner jaw on long rail (y≈0): at x ≈ 0.082m.
    // Position OB just inside the top rail, heading -x (toward TL corner).
    // The ball path passes through the jaw zone; at high speed it should
    // rattle off the near jaw and come to rest outside the throat zone.
    const W = 2.540;
    const H = 1.270;
    // OB in metres: (0.20, 0.029) — on the long rail, 0.20 m from TL corner.
    // CB in metres: (0.60, 0.080) — aims toward OB to send OB toward corner.
    final obRelX = 0.20 / W * (1 - 2 * 7 / 114) + 7 / 114;
    final obRelY = 0.029 / H * (1 - 2 * 7 / 64) + 7 / 64;
    final cbRelX = 0.60 / W * (1 - 2 * 7 / 114) + 7 / 114;
    final cbRelY = 0.080 / H * (1 - 2 * 7 / 64) + 7 / 64;

    // Aim in METRE-space (the physics sim uses metre coordinates for direction,
    // not relative coords which have a different x/y scale ratio).
    const aimDxM = 0.20 - 0.60; // -0.40 m
    const aimDyM = 0.029 - 0.080; // -0.051 m
    final aimLen = math.sqrt(aimDxM * aimDxM + aimDyM * aimDyM);

    final result = simulate(
      balls: [
        PhysicsBall(id: 'cb', x: cbRelX, y: cbRelY),
        PhysicsBall(id: 'ob', x: obRelX, y: obRelY),
      ],
      cueBallId: 'cb',
      shot: ShotParams(
        aimDir: Vec2(aimDxM / aimLen, aimDyM / aimLen),
        speedMs: 3.5, // firm shot — enough to reach the pocket at speed
        hitX: 0,
        hitY: 0,
      ),
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );

    final obPocketed = result.endPositions['ob'] == null;
    final obEnd = result.endPositions['ob'];
    final obPocketContacts = result.pocketContactDetails
        .where((contact) => contact.ballId == 'ob')
        .toList();
    // ignore: avoid_print
    print(
      '\nCorner-jaw rattle test: pocketed=$obPocketed  endPos=$obEnd '
      'contacts=${obPocketContacts.map((c) => '${c.pocket}/${c.surface}@${c.timeSec.toStringAsFixed(3)}').toList()}',
    );

    expect(
      obPocketed,
      isFalse,
      reason:
          'OB shot along long rail clips corner jaw — should rattle, not sink',
    );
  });

  test(
    'uncentered long-rail shot rattles before its centre enters the hole',
    () {
      const W = 2.540;
      const H = 1.270;
      final obM = const Vec2(0.40, 0.045);
      // A slight cross-table offset makes the object ball contact both facings
      // before its centre reaches the rendered black hole.
      const angle = -5.0 * math.pi / 180.0;
      final aim = Vec2(-math.cos(angle), math.sin(angle));
      final cbM = obM - aim * 0.30;
      final cb = _mToRel(cbM, W, H);
      final ob = _mToRel(obM, W, H);
      final result = simulate(
        balls: [
          PhysicsBall(id: 'cb', x: cb.x, y: cb.y),
          PhysicsBall(id: 'ob', x: ob.x, y: ob.y),
        ],
        cueBallId: 'cb',
        shot: ShotParams(aimDir: aim, speedMs: 2.4),
        tableConfig: PhysicsTableConfig.nineFootDiamond,
      );
      final contacts = result.pocketContactDetails
          .where((c) => c.ballId == 'ob' && c.pocket == 'topLeft')
          .toList();
      expect(result.endPositions['ob'], isNull);
      expect(result.pocketPositions['ob'], isNotNull);
      expect(
        contacts.length,
        greaterThanOrEqualTo(2),
        reason: 'the ball should contact both facings before reaching the hole',
      );
      expect(contacts[0].surface, isNot(contacts[1].surface));
    },
  );

  test(
    'faster shallow corner entry can rattle after repeated jaw contacts',
    () {
      const W = 2.540;
      const H = 1.270;
      const obM = Vec2(0.40, 0.045);
      // -7.5° sits inside the slow make window but just outside the fast one:
      // at the top of Dr. Dave's "fast" band the ball alternates between the
      // two facings and rattles out instead of dropping.
      const angle = -7.5 * math.pi / 180.0;
      final aim = Vec2(-math.cos(angle), math.sin(angle));
      final cbM = obM - aim * 0.30;
      final cb = _mToRel(cbM, W, H);
      final ob = _mToRel(obM, W, H);

      final softResult = simulate(
        balls: [
          PhysicsBall(id: 'cb', x: cb.x, y: cb.y),
          PhysicsBall(id: 'ob', x: ob.x, y: ob.y),
        ],
        cueBallId: 'cb',
        shot: ShotParams(aimDir: aim, speedMs: 1.8),
        tableConfig: PhysicsTableConfig.nineFootDiamond,
      );
      final fastResult = simulate(
        balls: [
          PhysicsBall(id: 'cb', x: cb.x, y: cb.y),
          PhysicsBall(id: 'ob', x: ob.x, y: ob.y),
        ],
        cueBallId: 'cb',
        shot: ShotParams(aimDir: aim, speedMs: _fastShotSpeedMs),
        tableConfig: PhysicsTableConfig.nineFootDiamond,
      );

      final fastContacts = fastResult.pocketContactDetails
          .where((contact) => contact.ballId == 'ob')
          .toList();
      expect(softResult.endPositions['ob'], isNull);
      expect(fastResult.endPositions['ob'], isNotNull);
      expect(
        fastContacts.length,
        greaterThanOrEqualTo(3),
        reason: 'the high-speed rattle should involve repeated facing contacts',
      );
    },
  );

  test(
    'shallow rail entry into a corner plays smaller as speed increases',
    () {
      // Dr. Dave: slower shots get bigger pockets. Sweep the aim of a shot
      // down the rail and count the contiguous band of aims that drop.
      const obM = Vec2(0.40, 0.045);
      int contiguousMadeAims(double speedMs) {
        var count = 0;
        for (var i = 0; i <= 40; i++) {
          final angle = -i * 0.25 * math.pi / 180.0;
          final aim = Vec2(-math.cos(angle), math.sin(angle));
          final cb = _mToRel(obM - aim * 0.30, 2.540, 1.270);
          final ob = _mToRel(obM, 2.540, 1.270);
          final result = simulate(
            balls: [
              PhysicsBall(id: 'cb', x: cb.x, y: cb.y),
              PhysicsBall(id: 'ob', x: ob.x, y: ob.y),
            ],
            cueBallId: 'cb',
            shot: ShotParams(aimDir: aim, speedMs: speedMs),
            tableConfig: PhysicsTableConfig.nineFootDiamond,
          );
          if (result.endPositions['ob'] != null) break;
          count++;
        }
        return count;
      }

      final windows =
          [1.2, 1.8, 2.5, _fastShotSpeedMs].map(contiguousMadeAims).toList();
      for (var i = 1; i < windows.length; i++) {
        expect(
          windows[i],
          lessThan(windows[i - 1]),
          reason: 'make window by speed should shrink: $windows',
        );
      }
    },
  );

  test(
    'hopping ball over a side pocket hole drops instead of passing over it',
    () {
      // A raised cue makes the cue ball hop about 10 mm (peak centre height
      // ~39 mm vs 28.6 mm at rest) as it crosses the side-pocket hole. Its
      // lowest point stays below the rail cap, so it strikes the back of the
      // pocket and drops. Previously capture required cloth contact, and the
      // ball passed over the hole and left the table.
      const W = 2.540;
      const H = 1.270;
      final cb = _mToRel(const Vec2(W / 2, 0.10), W, H);
      final result = simulate(
        balls: [PhysicsBall(id: 'cb', x: cb.x, y: cb.y)],
        cueBallId: 'cb',
        shot: const ShotParams(
          aimDir: Vec2(0, -1),
          speedMs: 3.0,
          elevationDeg: 15,
        ),
        tableConfig: const PhysicsTableConfig(playW: W, playH: H),
      );
      expect(result.peakHeights['cb']!, greaterThan(_ballRadius + 0.005));
      expect(result.endPositions['cb'], isNull);
      expect(result.pocketPositions['cb'], isNotNull);
    },
  );

  test(
    'corner pocket drops as soon as the ball center enters the black hole',
    () {
      const W = 2.540;
      const H = 1.270;
      const holeRadius = 2.5 * 0.0254;
      const shelfInches = 1.625;
      const mouthInches = 4.5;
      final holeOffset =
          (shelfInches + 2.5 - mouthInches / 2) / math.sqrt2 * 0.0254;
      // Inside the rendered hole, but not far enough for the ball's whole
      // diameter to clear its edge.
      final centerDistance = holeRadius - 0.010;
      final ballM = Vec2(
        -holeOffset + centerDistance / math.sqrt2,
        -holeOffset + centerDistance / math.sqrt2,
      );
      final ball = _mToRel(ballM, W, H);

      final result = simulate(
        balls: [PhysicsBall(id: 'ob', x: ball.x, y: ball.y)],
        cueBallId: 'ob',
        shot: ShotParams(aimDir: const Vec2(1, 0), speedMs: 0),
        tableConfig: PhysicsTableConfig.nineFootDiamond,
      );

      expect(
        result.endPositions['ob'],
        isNull,
        reason: 'Center inside the black hole must be pocketed immediately',
      );
      expect(
        result.frames.last.containsKey('ob'),
        isFalse,
        reason: 'The replay must show the ball disappear after dropping',
      );
    },
  );
}
