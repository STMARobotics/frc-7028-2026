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
    double target = ShooterSubsystem.chooseYawShortestDistance(Rotations.of(0.6), Rotations.of(-0.2));
    System.out.println("Target: " + target);
    assertEquals(-0.4, target, 1e-9);
  }

  @Test
  void chooseYawShortestDistanceRespectsLimits() {
    double target = ShooterSubsystem.chooseYawShortestDistance(Rotations.of(3.2), Rotations.of(0.1));

    assertEquals(0.2, target, 1e-9);
  }

}
