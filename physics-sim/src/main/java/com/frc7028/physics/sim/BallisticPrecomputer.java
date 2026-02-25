package com.frc7028.physics.sim;

import static edu.wpi.first.units.Units.Radians;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.frc7028.physics.sim.AdaptiveSystem.AdaptiveDebugObject;
import com.frc7028.physics.sim.AdaptiveSystem.AdaptiveOutput;
import com.frc7028.physics.sim.Debugger.DebugEventType;
import com.frc7028.physics.sim.Debugger.DebugType;
import com.frc7028.physics.sim.Integrator.forceInput;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import java.util.HashMap;
import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class BallisticPrecomputer {

  // Source - https://stackoverflow.com/a/5762502
  // Posted by WhiteFang34, modified by community. See post 'Timeline' for change history
  // Retrieved 2026-02-18, License - CC BY-SA 3.0

  private static final String ANSI_RESET = "\u001B[0m";
  private static final String ANSI_BLACK = "\u001B[30m";
  private static final String ANSI_RED = "\u001B[31m";
  private static final String ANSI_GREEN = "\u001B[32m";
  private static final String ANSI_YELLOW = "\u001B[33m";
  private static final String ANSI_BLUE = "\u001B[34m";
  private static final String ANSI_PURPLE = "\u001B[35m";
  private static final String ANSI_CYAN = "\u001B[36m";
  private static final String ANSI_WHITE = "\u001B[37m";

  // ^^^ use colorful outputs

  private DebugEventType lastDebugEvent;
  int iter = 0;

  public SimulatorResolution simulatorResolution;
  public IntegratorResolution integratorResolution;
  public BallisticEnvironmentProfile environmentProfile;
  public FieldMetrics fieldMetrics;
  private AdaptiveSystem<RobotState> adaptiveBallisticErrorSystem;
  public SimulatorVisualizer visualizer;

  BallisticProjectileState projectileState;

  Function<forceInput, Translation3d> forceFunction;

  Function<Double, Rotation3d> speedToSpin;

  Function<RobotState, ShotParameters> computeShotGuess;

  public record RobotState(Translation2d position, Translation2d velocity) {
  }

  public record SimulationResult(Translation3d endPosition, Translation3d closestPosition, double flightTime) {

  }

  private ShotParameters MatrixToParameters(SimpleMatrix parameters) {
    return new ShotParameters(Radians.of(parameters.get(0, 0)), Radians.of(parameters.get(1, 0)), parameters.get(2, 0));
  }

  public record ShotParameters(Angle yaw, Angle pitch, double speed) {
    public ShotParameters(Angle yaw, Angle pitch, double speed) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.speed = speed;
    }

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

    SimpleMatrix getInputMatrix() {
      double[] input = { this.yaw.baseUnitMagnitude(), this.pitch.baseUnitMagnitude(), this.speed };
      return new SimpleMatrix(input);
    }
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
      Function<RobotState, ShotParameters> computeShotGuess,
      SimulatorVisualizer visualizer) {
    this.simulatorResolution = simulatorResolution;
    this.integratorResolution = integratorResolution;
    this.environmentProfile = environmentProfile;
    this.fieldMetrics = fieldMetrics;

    this.visualizer = visualizer;

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

      acceleration = acceleration.plus(unitDirection.times(-0 * drag))
          .plus((new Translation3d(spinAxis.cross(stateVel))).times(0))
          .plus(new Translation3d(0, 0, environmentProfile.gravitationalAcceleration));
      return acceleration;
    };

    double[] deltas = {
        simulatorResolution.deltaAngle().baseUnitMagnitude(),
        simulatorResolution.deltaAngle().baseUnitMagnitude(),
        simulatorResolution.delta_dSpeed() };

    // error, error quantifier, input deltas, max iterations, input count, output count
    this.adaptiveBallisticErrorSystem = new AdaptiveSystem<RobotState>(
        (SimpleMatrix input, RobotState currentState) -> {
          iter++;
          ShotParameters attemptShotParameters = MatrixToParameters(input);

          SimulationResult simResult = this
              .simulateBall(this.projectileState, currentState, attemptShotParameters, visualizer);

          System.out.println(this.lastDebugEvent);
          // StringBuffer cvs = new StringBuffer("x_1, y_1, z_1");
          // this.projectileState.trajectory.forEach((Translation3d pos) -> {
          // cvs.append(String.format("%.6f,%.6f,%.6f%n", pos.getX(), pos.getY(), pos.getZ()));
          // });
          // Path outputPath = Paths.get("trajectory" + this.iter + ".csv");
          // try {
          // Files.writeString(outputPath, cvs.toString());
          // } catch (IOException e) {
          // }

          Translation3d closestErrorDisplacement = simResult.closestPosition
              .minus(this.fieldMetrics.targetRegion().center);
          Translation3d endErrorDisplacement = simResult.endPosition.minus(this.fieldMetrics.targetRegion().center);

          double[] errorVector = {
              endErrorDisplacement.getX(),
              endErrorDisplacement.getY(),
              endErrorDisplacement.getZ() };
          return new SimpleMatrix(errorVector);
        },
        (SimpleMatrix output) -> (double) output.normF(),
        (AdaptiveDebugObject dbo) -> {
          switch (dbo.type()) {
            case DebugType.Event: {
              this.lastDebugEvent = dbo.event();
              System.out.println(dbo.event().getColor() + dbo.type() + " | " + dbo.event() + ANSI_RESET);
            }
            case DebugType.Display: {
              if (dbo.matrixOutput() != this.adaptiveBallisticErrorSystem.defaultMatrix) {
                System.out.println(ANSI_WHITE + dbo.displayEvent() + " | " + dbo.matrixOutput() + ANSI_RESET);
              }
            }
          }
          ;
        },
        new SimpleMatrix(deltas),
        0,
        10,
        3,
        3);

    this.projectileState = new BallisticProjectileState(this, integratorResolution, forceFunction, visualizer);
  }

  public SimulationResult simulateBall(
      BallisticProjectileState simulatedProjectileState,
      RobotState state,
      ShotParameters parameters,
      SimulatorVisualizer visualizer) {
    Translation3d initialPosition = new Translation3d(
        state.position.getX(),
        state.position.getY(),
        this.fieldMetrics.startingHeight());
    Translation3d initialVelocity = new Translation3d(state.velocity.getX(), state.velocity.getY(), 0)
        .plus(anglesToUnitVec(parameters.yaw, parameters.pitch).times(parameters.speed));

    simulatedProjectileState.launch(initialPosition, initialVelocity, this.speedToSpin.apply(parameters.speed()));

    Translation3d targetPosition = this.fieldMetrics.targetRegion().center;

    Translation3d nearestPosition = this.fieldMetrics.targetRegion().center.minus(initialPosition);
    double nearestDistance = nearestPosition.getDistance(targetPosition);

    // Actually step the projectile over time, and stop when it either collides or is out of bounds
    while (simulatedProjectileState.completedTrajectory()) {
      simulatedProjectileState.step();
      simulatedProjectileState.trajectory.step(simulatedProjectileState.position);

      // System.out.println(simulatedProjectileState.position);

      Translation3d currentPosition = simulatedProjectileState.position;
      if (currentPosition.getDistance(targetPosition) < nearestDistance) {
        nearestPosition = currentPosition;
      }
    }

    simulatedProjectileState.trajectory.push();

    return new SimulationResult(
        simulatedProjectileState.position,
        nearestPosition,
        simulatedProjectileState.timeElapsed);

    // System.out.println(simulatedProjectileState.dt);
  }

  private double niceify(double x) {
    return Math.floor(x * 100) / 100d;
  };

  public ShotParameters convergeSolution(
      BallisticProjectileState simulatedProjectileState,
      RobotState state,
      ShotParameters parameters) {
    // https://pages.hmc.edu/ruye/MachineLearning/lectures/ch2/node7.html

    AdaptiveOutput convergedSolution = this.adaptiveBallisticErrorSystem
        .simpleConverge(parameters.getInputMatrix(), state);

    SimpleMatrix convergedInputs = convergedSolution.output().inputs();

    return MatrixToParameters(convergedInputs);
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