import 'dart:math' as math;

/// Single source of truth for the play-surface insets.
/// Matches the Diamond Pro-Am cushion face geometry (7-inch rail on a 114-inch top).
const double railWidth = 7.0 / 114.0; // ≈ 0.0614
const double railHeight = 7.0 / 64.0; // ≈ 0.1094

/// Diamond-style pocket geometry, shared by rendering and simulation.
const double cornerMouthInches = 4.5;
const double cornerHoleRadiusInches = 2.5;
const double cornerShelfInches = 1.625;
// TP B.15's worked geometry (4.5-inch mouth, 3.75-inch throat, 2-inch
// cushion face depth) gives a 38.323-degree face angle measured from the rail.
const double cornerFacingAngleDegrees = 38.323221901499416;
const double sideMouthInches = 5.0;
const double sideHoleRadiusInches = 2.5;
// TP 3.5 / BCA standard side-pocket shelf depth (0"–3/8" range; 3/16" nominal).
const double sideShelfInches = 0.1875;
const double sideFacingAngleDegrees = 13.0;
const double pocketCushionDepthInches = 2.25;
const double inchesToMeters = 0.0254;

/// Derive a corner-pocket face angle from the measurements in Dr. Dave TP B.15.
/// The returned acute angle is measured from the adjacent rail, matching the
/// convention used by the renderer and cushion-contact solver.
double cornerFacingAngleDegreesFromDimensions({
  required double mouthInches,
  required double throatInches,
  required double cushionFaceDepthInches,
}) {
  final mouthThroatDifference = mouthInches - throatInches;
  if (mouthThroatDifference <= 0 || cushionFaceDepthInches <= 0) {
    throw ArgumentError(
      'Mouth must exceed throat and cushion-face depth must be positive.',
    );
  }
  final undercutRadians = math.atan(
    mouthThroatDifference /
        (2 * math.sqrt2 * cushionFaceDepthInches + mouthThroatDifference),
  );
  return 45 - undercutRadians * 180 / math.pi;
}
