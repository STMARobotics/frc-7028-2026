package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.function.Function;

// https://math.stackexchange.com/questions/3839974/modelling-spin-using-the-magnus-effect

public class VectorState {

  public Translation3d position;
  public Translation3d velocity;
  public Rotation3d spin;
  public Integrator integrator;
  double dt;
  double timeElapsed;

  ArrayList<Translation3d> trajectory = new ArrayList<>();

  public VectorState(
      IntegratorResolution integratorResolution,
      Function<com.frc7028.physics.sim.Integrator.forceInput, Translation3d> forceFunction) {
    this.integrator = new Integrator(this, integratorResolution, forceFunction);
  }

  public void pose(Translation3d position) {
    this.position = position;
  }

  public void stepPosition() {
    this.position = this.position.plus(this.velocity.times(this.dt));
    this.trajectory.add(this.position);
  }

  public void step() {
    this.integrator.step();
    this.timeElapsed += this.dt;
  }
}
