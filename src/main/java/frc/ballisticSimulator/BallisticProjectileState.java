package frc.ballisticSimulator;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.ballisticSimulator.BallisticSimulator.RawBallisticLaunchConditions;

class BallisticProjectileState {
  BallisticSimulator simulator;
  Translation3d position;
  Translation3d velocity;
  Translation3d acceleration;
  AngularVelocity angularVelocity;
  BoundryState boundryState;
  CollisionStatus collisionStatus;
  double timeElapsed;
  Translation3d error;

  public void launch(
      Translation3d position,
      Translation3d velocity,
      Translation3d acceleration,
      AngularVelocity angularVelocity) {
    this.position = position;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.angularVelocity = angularVelocity;

    this.timeElapsed = 0;
  }

  public void launch(RawBallisticLaunchConditions startingConditions) {
    this.launch(
        startingConditions.position(),
          startingConditions.velocity(),
          Translation3d.kZero,
          startingConditions.angularVelocity());
  }

  public void stepVelocity() {

  }

  public void stepSpin() {

  }

  public void stepPosition() {

  }

  public void stepOrientation() {

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

  public void step() {
    stepVelocity();
    stepSpin();

    stepPosition();
    stepOrientation();

    this.timeElapsed += this.simulator.resolution.deltaTime;
  }

}