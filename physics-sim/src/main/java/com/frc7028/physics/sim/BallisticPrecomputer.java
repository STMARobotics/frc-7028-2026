package com.frc7028.physics.sim;

import static edu.wpi.first.units.Units.Radians;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.frc7028.physics.sim.Integrator.forceInput;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import java.util.HashMap;
import java.util.function.Function;

public class BallisticPrecomputer {

  public SimulatorResolution simulatorResolution;
  public IntegratorResolution integratorResolution;
  public BallisticEnvironmentProfile environmentProfile;
  public FieldMetrics fieldMetrics;

  BallisticProjectileState projectileState;

  Function<forceInput, Translation3d> forceFunction;

  Function<Double, Rotation3d> speedToSpin;

  Function<RobotState, ShotParameters> computeShotGuess;

  public record RobotState(Translation2d position, Translation2d velocity) {
  }

  public record ShotParameters(Angle yaw, Angle pitch, double speed) {
    ShotParameters stepYaw(Angle step) {
      return new ShotParameters(yaw.plus(step), pitch, speed);
    };

    ShotParameters stepPitch(Angle step) {
      return new ShotParameters(yaw, pitch.plus(step), speed);
    }

    ShotParameters stepSpeed(double step) {
      return new ShotParameters(yaw, pitch, speed + step);
    }

    ShotParameters stepAll(Angle stepYaw, Angle stepPitch, double stepSpeed) {
      return new ShotParameters(yaw.plus(stepYaw), pitch.plus(stepPitch), speed + stepSpeed);
    }

  }

  private record SimulationResult(Translation3d error, double errorYaw, double errorRadial, double errorHeight) {

  }

  private Translation3d anglesToUnitVec(Angle yaw, Angle pitch) {
    double yawRadians = yaw.in(Radians);
    double pitchRadians = pitch.in(Radians);
    return new Translation3d(
        cos(pitchRadians) * cos(yawRadians),
        cos(pitchRadians) * sin(yawRadians),
        sin(pitchRadians));
  };

  public BallisticPrecomputer(
      SimulatorResolution simulatorResolution,
      IntegratorResolution integratorResolution,
      BallisticEnvironmentProfile environmentProfile,
      FieldMetrics fieldMetrics,
      Function<Double, Rotation3d> speedToSpin,
      Function<RobotState, ShotParameters> computeShotGuess) {
    this.simulatorResolution = simulatorResolution;
    this.integratorResolution = integratorResolution;
    this.environmentProfile = environmentProfile;
    this.fieldMetrics = fieldMetrics;

    this.speedToSpin = speedToSpin;

    this.forceFunction = (forceInput projectileState) -> {
      Translation3d acceleration = Translation3d.kZero; // <0, 0, 0>
      Translation3d stateVel = projectileState.velocity();
      Rotation3d stateSpin = projectileState.spin();

      Translation3d spinAxis = new Translation3d(stateSpin.getAxis()).times(stateSpin.getAngle());

      Translation3d unitDirection = stateVel.div(Math.max(stateVel.getNorm(), 1e-4));

      double dynamicPressure = 0.5d * environmentProfile.airDensity * stateVel.getSquaredNorm();

      double drag = environmentProfile.ballisticProjectileDragCoefficient * dynamicPressure
          * environmentProfile.ballisticProjectileCrossSectionalArea;

      double magnus = (0.5d) * environmentProfile.airDensity * environmentProfile.ballisticProjectileCrossSectionalArea
          * environmentProfile.ballisticProjectileMagnusCoefficient;

      acceleration = acceleration.plus(unitDirection.times(-1 * drag))
          .plus((new Translation3d(spinAxis.cross(stateVel))).times(magnus))
          .plus(new Translation3d(0, 0, environmentProfile.gravitationalAcceleration));
      return acceleration;
    };

    this.computeShotGuess = computeShotGuess;

    this.projectileState = new BallisticProjectileState(this, integratorResolution, forceFunction);
  }

  private SimulationResult simulateBall(
      BallisticProjectileState simulatedProjectileState,
      RobotState state,
      ShotParameters parameters) {
    Translation3d initialPosition = new Translation3d(
        state.position.getX(),
        state.position.getY(),
        this.fieldMetrics.startingHeight());
    Translation3d initialVelocity = new Translation3d(state.velocity.getX(), state.velocity.getY(), 0)
        .plus(anglesToUnitVec(parameters.yaw, parameters.pitch).times(parameters.speed));

    // Launch the projectile
    simulatedProjectileState.launch(initialPosition, initialVelocity, this.speedToSpin.apply(parameters.speed()));

    // Actually step the projectile over time, and stop when it either collides or is out of bounds
    while (simulatedProjectileState.completedTrajectory()) {
      simulatedProjectileState.step();
    }

    Translation3d endPosition = simulatedProjectileState.position;
    Translation3d errorDisplacement = endPosition.minus(this.fieldMetrics.targetRegion().center);
    Translation2d errorDisplacement2D = errorDisplacement.toTranslation2d();

    return new SimulationResult(
        errorDisplacement,
        atan2(errorDisplacement2D.getY(), errorDisplacement2D.getX()),
        errorDisplacement2D.getNorm(),
        errorDisplacement.getZ());
  }

  public ShotParameters convergeSolution(
      BallisticProjectileState simulatedProjectileState,
      RobotState state,
      ShotParameters parameters) {
    // https://pages.hmc.edu/ruye/MachineLearning/lectures/ch2/node7.html

    int iterations = 0;

    while (iterations <= this.simulatorResolution.maxIterations()) {
      iterations++;

      SimulationResult errorPrior = simulateBall(simulatedProjectileState, state, parameters);

      double errRadialPrior = errorPrior.errorRadial();
      double errYawPrior = errorPrior.errorYaw();
      double errHeightPrior = errorPrior.errorHeight();

      if (errorPrior.error.getNorm() < this.simulatorResolution.targetMaxDistanceForConvergence()) {
        return parameters;
      }

      Angle deltaAngle = this.simulatorResolution.deltaAngle();
      double deltaSpeed = this.simulatorResolution.delta_dSpeed();

      double da = deltaAngle.baseUnitMagnitude();

      ShotParameters steppedYawParameters = parameters.stepYaw(deltaAngle);
      ShotParameters steppedPitchParameters = parameters.stepPitch(deltaAngle);
      ShotParameters steppedSpeedParameters = parameters.stepSpeed(deltaSpeed);

      SimulationResult steppedYaw = simulateBall(simulatedProjectileState, state, steppedYawParameters);
      SimulationResult steppedPitch = simulateBall(simulatedProjectileState, state, steppedPitchParameters);
      SimulationResult steppedSpeed = simulateBall(simulatedProjectileState, state, steppedSpeedParameters);

      // y(yaw); p(pitch); s(speed)
      // Ey(error yaw); Er(error radial); Ez(error height)

      // Compose jacobian [errorYaw, errorDistance(radial), errorHeight] x [dYaw, dPitch, dSpeed]

      double Eydy = (steppedYaw.errorYaw() - errYawPrior) / da;
      double Erdy = (steppedYaw.errorRadial() - errRadialPrior) / da;
      double Ezdy = (steppedYaw.errorHeight() - errHeightPrior) / da;

      double Eydp = (steppedPitch.errorYaw() - errYawPrior) / da;
      double Erdp = (steppedPitch.errorRadial() - errRadialPrior) / da;
      double Ezdp = (steppedPitch.errorHeight() - errHeightPrior) / da;

      double Eyds = (steppedSpeed.errorYaw() - errYawPrior) / deltaSpeed;
      double Erds = (steppedSpeed.errorRadial() - errRadialPrior) / deltaSpeed;
      double Ezds = (steppedSpeed.errorHeight() - errHeightPrior) / deltaSpeed;

      Matrix<N3, N3> ErrorJacobian = MatBuilder
          .fill(N3.instance, N3.instance, Eydy, Erdy, Ezdy, Eydp, Erdp, Ezdp, Eyds, Erds, Ezds);
      Matrix<N3, N1> baseError = MatBuilder.fill(N3.instance, N1.instance, errYawPrior, errRadialPrior, errHeightPrior);

      // May want to consider different method for "inverse" of matrix thats more efficent or stable. See
      // "psuedo-inverse"
      Matrix<N3, N1> parameterStepMatrix = ErrorJacobian.inv().times(baseError);

      Angle yawStep = Radians.of(-parameterStepMatrix.get(0, 0));
      Angle pitchStep = Radians.of(-parameterStepMatrix.get(1, 0));
      double speedStep = -parameterStepMatrix.get(2, 0);

      parameters = parameters.stepAll(yawStep, pitchStep, speedStep);
    }

    return parameters;
  }

  public HashMap<RobotState, ShotParameters> computeTable() {
    Region2d fieldPlane = this.fieldMetrics.fieldPlane();

    Translation2d minPos = fieldPlane.min;
    Translation2d maxPos = fieldPlane.max;

    HashMap<RobotState, ShotParameters> shotParameters = new HashMap<>();

    double dp = this.simulatorResolution.delta_dPosComp();
    double dv = this.simulatorResolution.delta_dVelComp();
    double mxVComp = this.simulatorResolution.maxVelComponent();

    for (double px = minPos.getX(); px <= maxPos.getX(); px += dp) {
      for (double py = minPos.getY(); py <= maxPos.getY(); py += dp) {
        for (double vx = -mxVComp; vx <= mxVComp; vx += dv) {
          for (double vy = -mxVComp; vy <= mxVComp; vy += dv) {
            RobotState currenRobotState = new RobotState(new Translation2d(px, py), new Translation2d(vx, vy));
            shotParameters.put(
                currenRobotState,
                  convergeSolution(
                      this.projectileState,
                        currenRobotState,
                        this.computeShotGuess.apply(currenRobotState)));
          }
        }
      }
    }

    return shotParameters;
  }
}
