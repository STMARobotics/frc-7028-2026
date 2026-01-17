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

  public 

  public Distance muzzleRadius;

  public BallisticShooter(
      Translation3d relativePosition,
      double maxProjectileSpeed,
      Angle[] yawRange,
      Angle[] pitchRange,
      Supplier<Angle> pitchSupplier,
      Supplier<Angle> yawSupplier,
      BallisticRobot robot) {

    this.relativePosition = relativePosition;
    this.maxProjectileSpeed = maxProjectileSpeed;
    this.pitchRange = pitchRange;
    this.yawRange = yawRange;
    this.pitchSupplier = pitchSupplier;
    this.yawSupplier = yawSupplier;
  };

  public Translation3d getRelativeMuzzlePosition() {
    double baseMuzzleRadius = muzzleRadius.baseUnitMagnitude();
    double pitch = this.pitchSupplier.get().in(Radians); // theta
    double yaw = this.yawSupplier.get().in(Radians) + this.; // phi

    return new Translation3d(
        baseMuzzleRadius * Math.cos(pitch) * Math.sin(yaw),
        baseMuzzleRadius * Math.sin(pitch) * Math.cos(yaw),
        baseMuzzleRadius * Math.sin(pitch));

  };

}
