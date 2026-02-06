package com.frc7028.physics.sim;

import com.frc7028.physics.sim.Integrator.forceInput;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Function;

class BallisticProjectileState extends VectorState {

  BallisticSimulator simulator;
  BoundryState boundryState;
  CollisionStatus collisionStatus;
  Translation3d error;
  Integrator ballisticIntegrator;

  public BallisticProjectileState(
      IntegratorResolution integratorResolution,
      Function<forceInput, Translation3d> forceFunction) {
    super(integratorResolution, forceFunction);
  }

  public void launch(Translation3d position, Translation3d velocity, Rotation3d spin) {
    this.position = position;
    this.velocity = velocity;
    this.spin = spin;

    this.integrator.reset();
  }

  public enum BoundryState {
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

  private BoundryState resolveBoundryState(BoundryState state) {
    this.boundryState = state;
    return state;
  }

  private CollisionStatus resolveCollision(CollisionStatus status) {
    this.collisionStatus = status;
    return status;
  }

  private boolean testOutOfBounds() {
    if (!this.simulator.resolution.fieldBounds.testPosition(position)) {
      resolveBoundryState(BoundryState.OUT_OF_BOUNDS);
      resolveCollision(CollisionStatus.FAILURE);
      return true;
    }
    return false;
  }

  private boolean testRejectedBoundries() {
    for (Region3d region : this.simulator.resolution.rejectRegions) {
      if (region.testPosition(position)) {
        resolveBoundryState(BoundryState.OUT_OF_BOUNDS);
        resolveCollision(CollisionStatus.FAILURE);
        return true;
      }
    }
    return false;
  }

  private boolean testTarget() {
    if (this.simulator.resolution.targetRegion.testPosition(position)) {
      resolveBoundryState(BoundryState.IN_BOUNDS);
      resolveCollision(CollisionStatus.TARGET);
      return true;
    }
    return false;
  }

  private boolean testObstacles() {
    for (Region3d obstacle : this.simulator.resolution.obstacleRegions) {
      if (obstacle.testPosition(position)) {
        resolveBoundryState(BoundryState.FAIL);
        resolveCollision(CollisionStatus.OBSTACLE);
        return true;
      }
    }
    return false;
  }

  public boolean testPosition() {
    return this.testOutOfBounds() || this.testRejectedBoundries() || this.testObstacles() || !this.testTarget();
  }

}