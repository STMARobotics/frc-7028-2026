package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

final class ShooterMath {
  private ShooterMath() {
  }

  /**
   * Wraps any rotation value into a heading range of [0, 1) rotations.
   *
   * @param rotations input heading in rotations (continuous or wrapped)
   * @return equivalent wrapped heading in [0, 1) rotations
   */
  static double wrapToUnitRotation(double rotations) {
    double wrapped = MathUtil.inputModulus(rotations, 0.0, 1.0);
    return wrapped >= 1.0 ? 0.0 : wrapped;
  }

  /**
   * Selects the legal equivalent of a wrapped heading that requires the least turret motion.
   *
   * <p>
   * The desired heading is treated as circular (desired + N turns), then the nearest equivalent
   * within soft limits is chosen relative to current continuous position.
   *
   * @param wrappedDesiredRotation desired heading in [0, 1) rotations
   * @param currentContinuousRotation current turret angle in continuous rotations
   * @param minSoftLimitRotation reverse soft limit in continuous rotations
   * @param maxSoftLimitRotation forward soft limit in continuous rotations
   * @return continuous target rotation to command
   */
  static double chooseYawShortestDistance(
      double wrappedDesiredRotation,
      double currentContinuousRotation,
      double minSoftLimitRotation,
      double maxSoftLimitRotation) {
    double normalizedDesired = wrapToUnitRotation(wrappedDesiredRotation);
    int nearestEquivalentIndex = (int) Math.rint(currentContinuousRotation - normalizedDesired);

    boolean foundCandidate = false;
    double bestCandidate = 0.0;
    double bestDistance = Double.POSITIVE_INFINITY;

    // Search a small neighborhood of equivalent headings (desired + N rotations)
    // and choose the legal candidate that requires the least motion.
    for (int equivalentIndex = nearestEquivalentIndex - 2; equivalentIndex <= nearestEquivalentIndex
        + 2; equivalentIndex++) {
      double candidateRotation = normalizedDesired + equivalentIndex;
      if (candidateRotation < minSoftLimitRotation || candidateRotation > maxSoftLimitRotation) {
        continue;
      }

      double distance = Math.abs(candidateRotation - currentContinuousRotation);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestCandidate = candidateRotation;
        foundCandidate = true;
      }
    }

    if (foundCandidate) {
      return bestCandidate;
    }

    double nearestEquivalent = normalizedDesired + Math.rint(currentContinuousRotation - normalizedDesired);
    return MathUtil.clamp(nearestEquivalent, minSoftLimitRotation, maxSoftLimitRotation);
  }
}
