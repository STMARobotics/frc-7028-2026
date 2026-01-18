package frc.robot.SOTOM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

public class BallisticRobotState {
  public BallisticShooter[] BallisticShooters;

  private Supplier<Translation2d> fieldPositionSupplier;
  private Supplier<Translation2d> fieldVelocitySupplier;
  private Supplier<Translation2d> fieldAccelerationSupplier;

  public Supplier<Angle> yawSupplier;

  /**
   * An abstraction of the robot which consolidates helpful physics information to assist in projectile physics
   * calculations
   * 
   * @param fieldPositionSupplier A 2D vector supplier for the robot's field position
   * @param fieldVelocitySupplier A 2D vector supplier for the robot's field velocity
   * @param fieldAccelerationSupplier A 2D vector supplier for the robot's field acceleration
   * @param yawSupplier The yaw of the robot WRT the field or global space
   */
  public BallisticRobotState(
      Supplier<Translation2d> fieldPositionSupplier,
      Supplier<Angle> yawSupplier,
      BallisticShooter... ballisticShooters) {
    this.fieldPositionSupplier = fieldPositionSupplier;
    this.yawSupplier = yawSupplier;
    this.BallisticShooters = ballisticShooters;
  }

  public Translation2d getFieldPosition() {
    return this.fieldPositionSupplier.get();
  }

  public Translation3d getFieldPosition3d() {
    Translation2d pos2d = this.fieldPositionSupplier.get();
    return new Translation3d(pos2d.getX(), pos2d.getY(), 0);
  }

  public Translation2d getFieldVelocity() {
    return this.fieldVelocitySupplier.get();
  }

  public Translation2d getFieldAcceleration() {
    return this.fieldAccelerationSupplier.get();
  }

  public Angle getYaw() {
    return this.yawSupplier.get();
  }
}