package com.frc7028.physics.sim;

import com.frc7028.physics.sim.Integrator.forceInput;
import com.frc7028.physics.sim.SimulatorVisualizer.Trajectory;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Function;

public class BallisticProjectileState extends VectorState {

  BallisticPrecomputer simulator;
  public BoundaryState boundaryState;
  public CollisionStatus collisionStatus;
  Translation3d error;
  Integrator ballisticIntegrator;
  private SimulatorVisualizer visualizer;
  public Trajectory trajectory;

  public BallisticProjectileState(
      BallisticPrecomputer simulator,
      IntegratorResolution integratorResolution,
      Function<forceInput, Translation3d> forceFunction,
      SimulatorVisualizer visualizer) {
    super(integratorResolution, forceFunction);
    this.visualizer = visualizer;
    this.simulator = simulator;
  }

  public void launch(Translation3d position, Translation3d velocity, Rotation3d spin) {
    this.reset();
    this.collisionStatus = CollisionStatus.NA;
    this.boundaryState = BoundaryState.IN_BOUNDS;
    this.position = position;
    this.velocity = velocity;
    this.spin = spin;
  }

  public void reset() {
    this.dt = this.integrator.integratorResolution.dtInitial();
    this.timeElapsed = 0;
    this.refreshTrajectory();
  }

  private void refreshTrajectory() {
    this.trajectory = visualizer.newTrajectory();
  }

  public enum BoundaryState {
    OUT_OF_BOUNDS, // It left the bounds of the match
    IN_BOUNDS, // It remains in the bounds of the match
    FAIL // It entered rejected bounds or obstacle bounds
  }

  public enum CollisionStatus {
    NA, // Ball is moving
    FAILURE, // Ball did not collide; it entered rejected boundries
    TARGET, // Ball collided with target
    OBSTACLE // Ball collided with obstacle
  }

  private BoundaryState resolveBoundaryState(BoundaryState state) {
    this.boundaryState = state;
    return state;
  }

  private CollisionStatus resolveCollision(CollisionStatus status) {
    this.collisionStatus = status;
    return status;
  }

  private boolean testOutOfBounds() {
    if (!this.simulator.fieldMetrics.fieldBounds().testPosition(position)) {
      resolveBoundaryState(BoundaryState.OUT_OF_BOUNDS);
      resolveCollision(CollisionStatus.FAILURE);
      return true;
    }
    return false;
  }

  private boolean testRejectedBoundaries() {
    for (Region3d region : this.simulator.fieldMetrics.rejectRegions()) {
      if (region.testPosition(position)) {
        resolveBoundaryState(BoundaryState.OUT_OF_BOUNDS);
        resolveCollision(CollisionStatus.FAILURE);
        return true;
      }
    }
    return false;
  }

  private boolean testTarget() {
    if (this.simulator.fieldMetrics.targetRegion().testPosition(position)) {
      resolveBoundaryState(BoundaryState.IN_BOUNDS);
      resolveCollision(CollisionStatus.TARGET);
      return true;
    }
    return false;
  }

  private boolean testObstacles() {
    for (Region3d obstacle : this.simulator.fieldMetrics.obstacles()) {
      if (obstacle.testPosition(position)) {
        resolveBoundaryState(BoundaryState.FAIL);
        resolveCollision(CollisionStatus.OBSTACLE);
        return true;
      }
    }
    return false;
  }

  public boolean completedTrajectory() {
    return !(this.testOutOfBounds() || this.testRejectedBoundaries() || this.testObstacles() || this.testTarget());
  }

}