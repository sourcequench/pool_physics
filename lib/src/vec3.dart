import 'dart:math' as math;

/// Small allocation-friendly 3-D vector used by the physics solver.
///
/// The diagram remains 2-D, but contact and angular state are not. Keeping
/// this type independent of Flutter's Offset makes the physics core usable by
/// Android, iOS, web, and future headless calibration tests.
class Vec3 {
  final double x;
  final double y;
  final double z;

  const Vec3(this.x, this.y, this.z);

  static const zero = Vec3(0, 0, 0);

  Vec3 operator +(Vec3 other) => Vec3(x + other.x, y + other.y, z + other.z);

  Vec3 operator -(Vec3 other) => Vec3(x - other.x, y - other.y, z - other.z);

  Vec3 operator -() => Vec3(-x, -y, -z);

  Vec3 operator *(double scale) => Vec3(x * scale, y * scale, z * scale);

  Vec3 operator /(double scale) => Vec3(x / scale, y / scale, z / scale);

  double dot(Vec3 other) => x * other.x + y * other.y + z * other.z;

  Vec3 cross(Vec3 other) => Vec3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x,
      );

  double get lengthSquared => dot(this);

  double get length => math.sqrt(lengthSquared);

  Vec3 normalized() {
    final l = length;
    return l > 0 ? this / l : zero;
  }

  @override
  String toString() => 'Vec3($x, $y, $z)';

  @override
  bool operator ==(Object other) =>
      other is Vec3 && x == other.x && y == other.y && z == other.z;

  @override
  int get hashCode => Object.hash(x, y, z);
}
