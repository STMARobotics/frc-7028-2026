package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/*
 * Unit tests for the ShooterSubsystem
 */
class ShooterSubsystemTest {
  @Test
  void normalizeToHalfTurnWrapsNegativeAndLargeValues() {
    assertEquals(-0.25, ShooterSubsystem.normalizeYaw(Rotations.of(-0.25)), 1e-9);
    assertEquals(0.2, ShooterSubsystem.normalizeYaw(Rotations.of(2.2)), 1e-9);
    assertEquals(0.0, ShooterSubsystem.normalizeYaw(Rotations.of(1.0)), 1e-9);
  }

  @Test
  void chooseYawShortestDistanceChoosesNearestLegalEquivalent() {
    double target = ShooterSubsystem
        .chooseYawShortestDistance(Rotations.of(0.2), Rotations.of(1.1), Rotations.of(-2.0), Rotations.of(2.0));

    assertEquals(1.2, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceRespectsSoftLimits() {
    double target = ShooterSubsystem
        .chooseYawShortestDistance(Rotations.of(0.2), Rotations.of(1.7), Rotations.of(-0.5), Rotations.of(0.5));

    assertEquals(0.2, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceFindsNearestEquivalentWithinHalfTurnBounds() {
    double target = ShooterSubsystem
        .chooseYawShortestDistance(Rotations.of(0.9), Rotations.of(4.3), Rotations.of(-0.5), Rotations.of(0.5));

    assertEquals(-0.1, target, 1e-9);
  }
}
