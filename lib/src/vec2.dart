import 'dart:math' as math;

/// An immutable 2-D vector for table-plane positions and directions.
///
/// Used throughout the simulation for ball positions and velocities in the
/// horizontal plane. Independent of Flutter's `Offset` so the physics core
/// compiles as a plain Dart package.
class Vec2 {
  final double x;
  final double y;

  const Vec2(this.x, this.y);

  static const Vec2 zero = Vec2(0, 0);

  Vec2 operator +(Vec2 other) => Vec2(x + other.x, y + other.y);
  Vec2 operator -(Vec2 other) => Vec2(x - other.x, y - other.y);
  Vec2 operator *(double s) => Vec2(x * s, y * s);
  Vec2 operator /(double s) => Vec2(x / s, y / s);
  Vec2 operator -() => Vec2(-x, -y);

  double dot(Vec2 other) => x * other.x + y * other.y;

  double get length => math.sqrt(x * x + y * y);

  /// Alias for [length]; both names appear in the codebase.
  double get distance => length;

  Vec2 get normalized {
    final l = length;
    return l > 1e-9 ? Vec2(x / l, y / l) : Vec2.zero;
  }

  @override
  bool operator ==(Object other) =>
      other is Vec2 && x == other.x && y == other.y;

  @override
  int get hashCode => Object.hash(x, y);

  @override
  String toString() => 'Vec2($x, $y)';
}
