package frc.ballisticSimulator;

import static edu.wpi.first.units.Units.Radians;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.HashMap;

class BallisticSimulator {
  BallisticEnvironmentProfile environmentProfile;
  BallisticSimulatorResolutionProfile resolution;
  BallisticProjectileState projectileState;

  Translation3d gravityAccelerationVector;

  public record BallisticCondition(double px, double py, double vx, double vy) {
  };

  private record BallisticParameters(Angle yaw, Angle pitch, double speed) {

  };

  private record BallisticSolution(Angle yaw, Angle pitch, double speed) {
  };

  public record RawBallisticLaunchConditions(
      Translation3d position,
      Translation3d velocity,
      AngularVelocity angularVelocity) {
  };

  public BallisticSimulator(
      BallisticEnvironmentProfile environmentProfile,
      BallisticSimulatorResolutionProfile resolutionProfile) {
    this.environmentProfile = environmentProfile;
    this.resolution = resolutionProfile;
    this.projectileState = new BallisticProjectileState();

    this.gravityAccelerationVector = new Translation3d(0, 0, this.environmentProfile.gravitationalAcceleration);
  }

  private Translation3d calculateDrag() {

  }

  private Translation3d calculateMagnus() {

  }

  private AngularVelocity calculateInitialAngularVelocity() {

  }

  private BallisticParameters calculateStationaryShot(BallisticCondition condition) {

  }

  private void stepBallistic() {

  }

  private Translation3d sphereAnglesToUnitVector(Angle yaw, Angle pitch) {
    double yawRadians = yaw.in(Radians);
    double pitchRadians = pitch.in(Radians);
    return new Translation3d(
        cos(pitchRadians) * cos(yawRadians),
        cos(pitchRadians) * sin(yawRadians),
        sin(pitchRadians));
  }

  private Translation3d parametersToVector(BallisticParameters parameters) {
    return sphereAnglesToUnitVector(parameters.yaw(), parameters.pitch()).times(parameters.speed());
  }

  private Translation3d simulateBall(BallisticCondition ballCondition, BallisticParameters parameters) {
    RawBallisticLaunchConditions rawLaunchConditions = new RawBallisticLaunchConditions(
        new Translation3d(ballCondition.px, ballCondition.py, this.resolution.startingHeight),
        parametersToVector(parameters).plus(new Translation3d(ballCondition.vx, ballCondition.vy, 0)),
        calculateInitialAngularVelocity());
    this.projectileState.launch(rawLaunchConditions);

    while (this.projectileState.testPosition()) {
      this.projectileState.step();
    }

    projectileState.error = this.projectileState.position.minus(this.resolution.targetPosition);
    return projectileState.error;
  }

  private BallisticParameters convergeSolution(BallisticCondition ballCondition, BallisticParameters ballParameters) {
    Angle yaw = ballParameters.yaw();
    Angle pitch = ballParameters.pitch();
    double speed = ballParameters.speed();

    Angle deltaAngle = this.resolution.deltaAngle;
    double deltaAngleRadians = deltaAngle.baseUnitMagnitude();
    double deltaSpeed = this.resolution.deltaSpeed;

    Translation3d errorPrior = simulateBall(ballCondition, ballParameters);

    if (errorPrior.getSquaredNorm() < this.resolution.convergenceThreshold) {
      return new BallisticParameters(yaw, pitch, speed);
    }

    BallisticParameters steppedYawParameters = new BallisticParameters(yaw.plus(deltaAngle), pitch, speed);

    BallisticParameters steppedPitchParameters = new BallisticParameters(yaw, pitch.plus(deltaAngle), speed);

    BallisticParameters steppedSpeedParameters = new BallisticParameters(yaw, pitch, speed + deltaSpeed);

    Translation3d dErrordYaw = simulateBall(ballCondition, steppedYawParameters).minus(errorPrior);
    Translation3d dErrordPitch = simulateBall(ballCondition, steppedPitchParameters).minus(errorPrior);
    Translation3d dErrordSpeed = simulateBall(ballCondition, steppedSpeedParameters).minus(errorPrior);

    double dYdx = dErrordYaw.getX() / deltaAngleRadians;
    double dYdy = dErrordYaw.getY() / deltaAngleRadians;
    double dYdz = dErrordYaw.getZ() / deltaAngleRadians;

    double dPdx = dErrordPitch.getX() / deltaAngleRadians;
    double dPdy = dErrordPitch.getY() / deltaAngleRadians;
    double dPdz = dErrordPitch.getZ() / deltaAngleRadians;

    double dSdx = dErrordSpeed.getX() / deltaSpeed;
    double dSdy = dErrordSpeed.getY() / deltaSpeed;
    double dSdz = dErrordSpeed.getZ() / deltaSpeed;

    Matrix<N3, N3> Jacobian = MatBuilder
        .fill(N3.instance, N3.instance, dYdx, dYdy, dYdz, dPdx, dPdy, dPdz, dSdx, dSdy, dSdz);

    Matrix<N3, N1> e = MatBuilder
        .fill(N3.instance, N1.instance, errorPrior.getX(), errorPrior.getY(), errorPrior.getZ());

    Matrix<N3, N1> step = Jacobian.inv().times(e);

    double yawStep = step.get(1, 1);
    double pitchStep = step.get(2, 1);
    double speedStep = step.get(3, 1);

    Angle newYaw = yaw.plus(Radians.of(yawStep));
    Angle newPitch = pitch.plus(Radians.of(pitchStep));
    double newSpeed = speed + speedStep;

    return convergeSolution(ballCondition, new BallisticParameters(newYaw, newPitch, newSpeed));
  }

  public void generateTable() {
    Region2d field = this.resolution.fieldPlane;
    double deltaPositionalComponent = this.resolution.deltaPositionalComponent;
    double deltaVelocityComponent = this.resolution.deltaVelocityComponent;
    double maxFieldVelocityComponent = this.resolution.maxFieldVelocityComponent;

    HashMap<BallisticCondition, BallisticParameters> table = new HashMap<>();

    for (double px = field.min.getX(); px <= field.max.getX(); px += deltaPositionalComponent) {
      for (double py = field.min.getY(); py <= field.max.getY(); py += deltaPositionalComponent) {
        for (double vx = -maxFieldVelocityComponent; vx <= maxFieldVelocityComponent; vx += deltaVelocityComponent) {
          for (double vy = -maxFieldVelocityComponent; vy <= maxFieldVelocityComponent; vy += deltaVelocityComponent) {
            BallisticCondition currentCondition = new BallisticCondition(px, py, vx, vy);
            BallisticParameters basicGuess = calculateStationaryShot(currentCondition);
            BallisticParameters convergedSolution = convergeSolution(currentCondition, basicGuess);
            table.put(currentCondition, convergedSolution);
          }
        }
      }
    }

  }

  public void main() {
  }
}