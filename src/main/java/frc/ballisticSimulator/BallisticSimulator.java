package frc.ballisticSimulator;

import static edu.wpi.first.units.Units.Radians;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import java.util.HashMap;

class BallisticSimulator {
  BallisticEnvironmentProfile environmentProfile;
  BallisticSimulatorResolutionProfile resolution;
  BallisticProjectileState projectileState;

  public record BallisticCondition(double px, double py, double vx, double vy) {
  };

  private record BallisticParameters(Angle yaw, Angle pitch, double speed) {

  };

  private record BallisticSolution(Angle yaw, Angle pitch, double speed) {
  };

  public record RawBallisticLaunchConditions(Translation3d position, Translation3d velocity, Rotation3d spin) {
  };

  public BallisticSimulator(
      BallisticEnvironmentProfile environmentProfile,
      BallisticSimulatorResolutionProfile resolutionProfile,
      IntegratorResolution integratorResolution) {
    this.environmentProfile = environmentProfile;
    this.resolution = resolutionProfile;
    this.projectileState = new BallisticProjectileState(integratorResolution, (Integrator.forceInput input) -> {
      Translation3d acceleration = Translation3d.kZero;
      Translation3d velocityUnitDirection = input.velocity().div(1 / input.velocity().getNorm());
      double drag = (0.5) * this.environmentProfile.ballisticProjectileDragCoefficient
          * this.environmentProfile.airDensity * input.velocity().getSquaredNorm();

      Translation3d axis = input.spin().getAxis().times(input.spin().getAngle())

      Translation3d liftAcceleration = Translation3d.cross(input.spin().getAxis(), velocityUnitDirection);

      return acceleration;
    });
  }

  private BallisticParameters calculateStationaryShot(BallisticCondition condition) {

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
        calculateInitialSpin());
    this.projectileState.launch(rawLaunchConditions);

    while (this.projectileState.testPosition()) {
      this.projectileState.step();
    }

    projectileState.error = this.projectileState.position.minus(this.resolution.targetPosition);
    return projectileState.error;
  }

  private BallisticParameters convergeSolution(BallisticCondition ballCondition, BallisticParameters ballParameters) {
    // https://pages.hmc.edu/ruye/MachineLearning/lectures/ch2/node7.html
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

    double dxdY = dErrordYaw.getX() / deltaAngleRadians;
    double dydY = dErrordYaw.getY() / deltaAngleRadians;
    double dzdY = dErrordYaw.getZ() / deltaAngleRadians;

    double dxdP = dErrordPitch.getX() / deltaAngleRadians;
    double dydP = dErrordPitch.getY() / deltaAngleRadians;
    double dzdP = dErrordPitch.getZ() / deltaAngleRadians;

    double dxdS = dErrordSpeed.getX() / deltaSpeed;
    double dydS = dErrordSpeed.getY() / deltaSpeed;
    double dzdS = dErrordSpeed.getZ() / deltaSpeed;

    Matrix<N3, N3> Jacobian = MatBuilder
        .fill(N3.instance, N3.instance, dxdY, dydY, dzdY, dxdP, dydP, dzdP, dxdS, dydS, dzdS);

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