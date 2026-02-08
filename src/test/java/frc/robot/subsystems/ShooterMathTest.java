package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShooterMathTest {
  @Test
  void wrapToUnitRotationWrapsNegativeAndLargeValues() {
    assertEquals(0.75, ShooterMath.wrapToUnitRotation(-0.25), 1e-9);
    assertEquals(0.2, ShooterMath.wrapToUnitRotation(2.2), 1e-9);
    assertEquals(0.0, ShooterMath.wrapToUnitRotation(1.0), 1e-9);
  }

  @Test
  void chooseYawShortestDistanceChoosesNearestLegalEquivalent() {
    double target = ShooterMath.chooseYawShortestDistance(0.9, 1.1, -2.0, 2.0);

    assertEquals(0.9, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceRespectsSoftLimits() {
    double target = ShooterMath.chooseYawShortestDistance(0.2, 1.7, -0.5, 0.5);

    assertEquals(0.2, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceFallsBackToClampedNearestEquivalent() {
    double target = ShooterMath.chooseYawShortestDistance(0.9, 4.3, -0.5, 0.5);

    assertEquals(0.5, target, 1e-9);
  }
}
