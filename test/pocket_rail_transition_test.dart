import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart' as sim;
import 'package:pool_physics/pool_physics.dart';

void main() {
  test('corner jaw does not replace the straight rail beside its nose', () {
    final result = sim.simulate(
      balls: const [PhysicsBall(id: 'cue', x: .1975, y: .7278)],
      cueBallId: 'cue',
      shot: const sim.ShotParams(aimDir: Vec2(-.9104, .4138), speedMs: 2.6822),
      tableConfig: PhysicsTableConfig.nineFootDiamond,
    );

    expect(result.cushionContactDetails, isNotEmpty);
    expect(result.cushionContactDetails.first.rail, 'left');
    expect(result.cushionContactDetails.first.timeSec, lessThan(.2));
    expect(
      result.pocketContactDetails.where((contact) => contact.timeSec < .2),
      isEmpty,
      reason: 'the tangent contact is on the rail, not the jaw point',
    );
  });
}
