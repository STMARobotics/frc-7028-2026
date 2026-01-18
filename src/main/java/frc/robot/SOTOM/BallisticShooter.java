package frc.robot.SOTOM;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

public class BallisticShooter {
  public Translation3d relativePosition;
  public double maxProjectileSpeed;
  public Angle[] yawRange;
  public Angle[] pitchRange;

  public Supplier<Angle> pitchSupplier;
  public Supplier<Angle> yawSupplier;

  public BallisticRobotState robot;

  public Distance muzzleRadius;

  public BallisticShooter(
      Translation3d relativePosition,
      double maxProjectileSpeed,
      Angle[] yawRange,
      Angle[] pitchRange,
      Supplier<Angle> pitchSupplier,
      Supplier<Angle> yawSupplier,
      BallisticRobotState robot) {

    this.relativePosition = relativePosition;
    this.maxProjectileSpeed = maxProjectileSpeed;
    this.pitchRange = pitchRange;
    this.yawRange = yawRange;
    this.pitchSupplier = pitchSupplier;
    this.yawSupplier = yawSupplier;
  };

  /**
   * 
   * @param relativePosition The position of the ballistic shooter W.R.T the robot, i.e., origin = robot
   * 
   */
  public Translation3d getMuzzlePositionWRTFieldRobot() {
    double baseMuzzleRadiusMeters = muzzleRadius.baseUnitMagnitude();
    double pitch = this.pitchSupplier.get().in(Radians); // theta
    double yaw = this.yawSupplier.get().in(Radians) + this.robot.yawSupplier.get().in(Radians); // phi

    return new Translation3d(
        baseMuzzleRadiusMeters * Math.cos(pitch) * Math.cos(yaw),
        baseMuzzleRadiusMeters * Math.cos(pitch) * Math.sin(yaw),
        baseMuzzleRadiusMeters * Math.sin(pitch));
  };

}
