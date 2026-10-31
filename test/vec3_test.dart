import 'package:test/test.dart';
import 'package:pool_physics/pool_physics.dart';

void main() {
  test('Vec3 dot and cross products', () {
    const x = Vec3(1, 0, 0);
    const y = Vec3(0, 1, 0);
    expect(x.dot(y), 0);
    expect(x.cross(y), const Vec3(0, 0, 1));
  });

  test('Vec3 normalization preserves direction', () {
    const v = Vec3(3, 4, 0);
    expect(v.length, closeTo(5, 1e-12));
    final unit = v.normalized();
    expect(unit.x, closeTo(0.6, 1e-12));
    expect(unit.y, closeTo(0.8, 1e-12));
    expect(unit.z, 0);
  });
}
