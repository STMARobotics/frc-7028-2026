package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShooterMathTest {
  @Test
  void wrapToUnitRotationWrapsNegativeAndLargeValues() {
    assertEquals(-0.25, ShooterSubsystem.wrapToUnitRotation(-0.25), 1e-9);
    assertEquals(0.2, ShooterSubsystem.wrapToUnitRotation(2.2), 1e-9);
    assertEquals(0.0, ShooterSubsystem.wrapToUnitRotation(1.0), 1e-9);
  }

  @Test
  void chooseYawShortestDistanceChoosesNearestLegalEquivalent() {
    double target = ShooterSubsystem.chooseYawShortestDistance(0.2, 1.1, -2.0, 2.0);

    assertEquals(1.2, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceRespectsSoftLimits() {
    double target = ShooterSubsystem.chooseYawShortestDistance(0.2, 1.7, -0.5, 0.5);

    assertEquals(0.2, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceFindsNearestEquivalentWithinHalfTurnBounds() {
    double target = ShooterSubsystem.chooseYawShortestDistance(0.9, 4.3, -0.5, 0.5);

    assertEquals(-0.1, target, 1e-9);
  }
}
