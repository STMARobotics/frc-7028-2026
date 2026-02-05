package frc.ballisticSimulator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.ballisticSimulator.Integrator.forceInput;
import java.util.function.Function;

// https://math.stackexchange.com/questions/3839974/modelling-spin-using-the-magnus-effect

public class VectorState {

  Translation3d position;
  Translation3d velocity;
  Rotation3d spin;
  Integrator integrator;
  double dt;
  double timeElapsed;

  public VectorState(IntegratorResolution integratorResolution, Function<forceInput, Translation3d> forceFunction) {
    this.integrator = new Integrator(this, integratorResolution, forceFunction);
  }

  public void pose(Translation3d position) {
    this.position = position;
  }

  public void stepPosition() {
    this.position = this.position.plus(this.velocity.times(this.dt));
  }

  public void step() {
    this.integrator.step();
    this.timeElapsed += this.dt;
  }
}
