package frc.robot.SOTOM;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.function.Function;

public class BallisticCalculator {

  public record InterpolatedPV(Angle angle, double velocity) {
  };

  public record CalculatedShot(Angle pitch, double velocity, double timeToTarget) {
  };

  public BallisticRobotState robotState;
  private Function<Double, InterpolatedPV> interpolator;

  BallisticCalculator(BallisticRobotState ballisticRobotState, Function<Double, InterpolatedPV> interpolator) {
    this.robotState = ballisticRobotState;
    this.interpolator = interpolator;
  };

  public Distance calculateFieldDistanceToTarget(
      Translation2d target,
      BallisticRobotState robot,
      BallisticShooter shooter) {
    return Meters.of(target.getDistance(shooter.getMuzzlePositionWTRField().toTranslation2d()));
  };

  Translation2d calculatePerpendicular(Translation2d translation) {
    return new Translation2d(-translation.getY(), translation.getX());
  }

  public Time calculateBallisticTimeToTarget(
      Distance fieldDistanceToTarget,
      Distance heightDifferential,
      InterpolatedPV shotParameters) {
    double v = shotParameters.velocity;
    double v_y = Math.sin(shotParameters.angle().baseUnitMagnitude()) * v;
    double d_y = heightDifferential.baseUnitMagnitude();
    double g = 9.81;
    return Seconds.of((v_y + Math.sqrt(Math.pow(v_y, 2) + 2 * g * d_y)) / g);
  };

  public Translation2d calculateShooterTangentialVelocity(BallisticRobotState robot, BallisticShooter shooter) {

    ChassisSpeeds robotChassisSpeeds = robot.getChasisSpeeds();

    double robotOmegaRadians = robotChassisSpeeds.omegaRadiansPerSecond;
    double shooterOmegaRadians = shooter.getShooterYawSpeed().baseUnitMagnitude();

    return calculatePerpendicular(shooter.getFieldShooterPosition()).times(robotOmegaRadians)
        .plus(
            calculatePerpendicular(shooter.getMuzzlePositionWRTShooter().toTranslation2d())
                .times(robotOmegaRadians + shooterOmegaRadians));

  };
}
