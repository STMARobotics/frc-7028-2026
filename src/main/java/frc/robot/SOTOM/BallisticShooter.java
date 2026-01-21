package frc.robot.SOTOM;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

public class BallisticShooter {
  public Translation3d relativePosition;
  public Translation2d relativeXYPosition;
  public double maxProjectileVelocity;
  public Angle yawRange;
  public Angle pitchRange;

  public Supplier<Angle> pitchSupplier;
  public Supplier<Angle> yawSupplier;

  Supplier<AngularVelocity> yawRateSupplier;

  public BallisticRobotState robotState;

  public Distance muzzleRadius;

  /**
   * An abstraction of a ballistic shooter object used for physics calculations
   * 
   * @param relativePosition The offset the shooter is W.R.T the robot
   * @param maxProjectileVelocity The max velocity (magnitude) the projectile is allowed to leave the muzzel at
   * @param yawRange The max yaw the shooter can rotate in <b>both directions</b>
   * @param pitchRange The max pitch the shooter can roate in <b>both directions</b>
   * @param pitchSupplier A function which returns the current pitch of the shooter, <b>NOT W.R.T the field</b>
   * @param yawSupplier A function which returns the current yaw of the shooter, <b> NOT W.R.T the field</b>
   * @param robot The respective robotState
   */

  public BallisticShooter(
      Translation3d relativePosition,
      double maxProjectileVelocity,
      Angle yawRange,
      Angle pitchRange,
      Supplier<Angle> pitchSupplier,
      Supplier<Angle> yawSupplier,
      Supplier<AngularVelocity> yawRateSupplier,
      BallisticRobotState robotState) {

    this.relativePosition = relativePosition;
    this.maxProjectileVelocity = maxProjectileVelocity;
    this.pitchRange = pitchRange;
    this.yawRange = yawRange;
    this.pitchSupplier = pitchSupplier;
    this.yawSupplier = yawSupplier;
    this.yawRateSupplier = yawRateSupplier;
    this.robotState = robotState;

    this.relativeXYPosition = relativePosition.toTranslation2d();
  };

  public Angle getYaw() {
    return this.yawSupplier.get();
  }

  public Angle getFieldYaw() {
    return this.robotState.getYaw().plus(this.getYaw());
  }

  /**
   * 
   * @return The translation or relative offset of the muzzle W.R.T the shooter
   */
  public Translation3d getMuzzlePositionWRTShooter() {
    double baseMuzzleRadiusMeters = muzzleRadius.baseUnitMagnitude();
    double pitch = this.pitchSupplier.get().in(Radians); // theta
    double yaw = this.getFieldYaw().in(Radians); // phi

    return new Translation3d(
        baseMuzzleRadiusMeters * Math.cos(pitch) * Math.cos(yaw),
        baseMuzzleRadiusMeters * Math.cos(pitch) * Math.sin(yaw),
        baseMuzzleRadiusMeters * Math.sin(pitch));
  };

  public Translation3d getMuzzlePositionWRTRobot() {
    return this.relativePosition.plus(this.getMuzzlePositionWRTShooter());
  }

  public Translation3d getMuzzlePositionWTRField() {
    return this.robotState.getFieldPosition3d().plus(this.getMuzzlePositionWRTRobot());
  }

  public AngularVelocity getShooterYawSpeed() {
    return this.yawRateSupplier.get();
  }

  public Translation2d getFieldShooterPosition() {
    return this.relativeXYPosition.rotateBy(new Rotation2d(this.robotState.getYaw()));
  }
}
