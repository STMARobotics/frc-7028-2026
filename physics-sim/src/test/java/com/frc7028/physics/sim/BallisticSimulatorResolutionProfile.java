package frc.ballisticSimulator;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;

class BallisticSimulatorResolutionProfile {
  double deltaTime;
  Angle deltaAngle;
  double deltaSpeed;
  double deltaPositionalComponent;
  double deltaVelocityComponent;
  double maxFieldVelocityComponent;
  double startingHeight;

  Region2d fieldPlane;
  ArrayList<Region3d> obstacleRegions;
  ArrayList<Region3d> rejectRegions;
  Region3d fieldBounds;
  Region3d targetRegion;
  Translation3d targetPosition;
  double convergenceThreshold;

  public BallisticSimulatorResolutionProfile(
      double deltaTime,
      Angle deltaAngle,
      double deltaSpeed,
      double deltaPositionalComponent,
      double deltaVelocityComponent,
      double maxFieldVelocityComponent,
      double startingHeight,
      Region2d fieldPlane,
      ArrayList<Region3d> obstacleRegions,
      ArrayList<Region3d> rejectRegions,
      Region3d targetRegion,
      Region3d fieldBounds,
      double convergenceThreshold) {
    this.deltaTime = deltaTime;
    this.deltaAngle = deltaAngle;
    this.deltaSpeed = deltaSpeed;
    this.deltaPositionalComponent = deltaPositionalComponent;
    this.deltaVelocityComponent = deltaVelocityComponent;
    this.maxFieldVelocityComponent = maxFieldVelocityComponent;
    this.startingHeight = startingHeight;

    this.convergenceThreshold = convergenceThreshold;

    this.fieldPlane = fieldPlane;
    this.obstacleRegions = obstacleRegions;
    this.fieldBounds = fieldBounds;
    this.targetRegion = targetRegion;
    this.targetPosition = targetRegion.min.plus(targetRegion.max.minus(targetRegion.min).times(1 / 2)); // This should
                                                                                                        // be a slim
                                                                                                        // rectangular
                                                                                                        // prism where
                                                                                                        // if the ball
                                                                                                        // enters this
                                                                                                        // region,
    // its assured to land in/on the target
  }
}