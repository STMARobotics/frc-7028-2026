package frc.robot.SOTOM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

public class BallisticRobotState {
  public BallisticShooter[] BallisticShooters;

  public Supplier<Translation2d> fieldPositionSupplier;
  public Supplier<Angle> yawSupplier;

  /**
   * An abstraction of the robot which consolidates helpful physics information to assist in projectile physics
   * calculations
   * 
   * @param fieldPositionSupplier The XY coordinates of the robot on the field
   * @param yawSupplier The yaw of the robot WRT the field or global space
   */
  public BallisticRobotState(Supplier<Translation2d> fieldPositionSupplier, Supplier<Angle> yawSupplier) {
    this.fieldPositionSupplier = fieldPositionSupplier;
    this.yawSupplier = yawSupplier;
  }
}