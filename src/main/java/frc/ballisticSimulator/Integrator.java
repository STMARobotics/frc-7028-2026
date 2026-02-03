/*
 * file: Integrator.java
 * date: 2/2/2026
 * description: A high precision simulator for solving dv/dt = f(v) employing RK45
 */
package frc.ballisticSimulator;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

// A way to solve a curve defined by F(t, v)
// da/dt = f(v)

// Recall: F = ma, i.e, force = mass * acceleration 

// In this ballistic simulation, we have forces like drag and lift in terms like this:
// da/dt = f(v) but
// we also define acceleration as the rate of change of velocity: dv/dt = acceleration

// So when we want to know the position of a ball over time and factor in drag etc., we run into a problem:
// v -> v + a(...)*dt

// if we increment velocity by a function of velocity, like drag, then it requires us to know the velocity prior to calculate a new velocity
// This requires us to simulate the velocity to know position, and for something sensitive like drag, we need a GOOD simulation. 
// Eulers method is "first order" way to solve this differential equation, it is decent, but it does behave well with sensitive or rapidly changing functions

// We use RK45 to accurate simulate the position, and it also dynamically modulates your time steps to ensure precision when the second derivative(f'') is large, that is to say f is very sensitive, and to increase it when smaller steps are not needed.

// spotless:off
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method
// We ignore time here these forces are not in terms of time, just velocity
public class Integrator {
  // K Coefficients 

// spotless:on
  double dt;
  double epsilon;

  private record VectorState(Translation3d Velocity) {
  };

  Function<Translation3d, Translation3d> accelerationFunction;
  Supplier<VectorState> VectorStateSupplier;
  Consumer<VectorState> VectorStateUpdater;

  private Translation3d f(Translation3d vel) {
    return this.accelerationFunction.apply(vel);
  }

  public Integrator(
      double dt,
      Function<Translation3d, Translation3d> accelerationFunction,
      Supplier<VectorState> vectorStatSupplier,
      Consumer<VectorState> VectorStateUpdater,
      double epsilon) {
    this.dt = dt;
    this.epsilon = epsilon;
    this.accelerationFunction = accelerationFunction;
    this.VectorStateSupplier = vectorStatSupplier;
    this.VectorStateUpdater = VectorStateUpdater;

  };

  private static final double[][] C = {
      { 1 / 4 },
      { 3 / 32, 9 / 32 },
      { 1932 / 2197, -7200 / 2197, 7296 / 2197 },
      { 439 / 216, -8, 3680 / 513, -845 / 4104 },
      { -8 / 27, 2, -3544 / 2565, 1859 / 4104, -11 / 40 } };

  // K Weights
  private static final double[][] W = {
      { 25 / 216, 0, 1408 / 2565, 2197 / 4104, -1 / 5 },
      { 16 / 135, 0, 6656 / 12825, 28561 / 56430, 9 / 50, 2 / 55 } };

  public void step() {
    VectorState initialVectorState = VectorStateSupplier.get();
    var v = initialVectorState.Velocity();

    double C11 = C[1 - 1][1 - 1] * dt;

    double C21 = C[2 - 1][1 - 1] * dt;
    double C22 = C[2 - 1][2 - 1] * dt;

    double C31 = C[3 - 1][1 - 1] * dt;
    double C32 = C[3 - 1][2 - 1] * dt;
    double C33 = C[3 - 1][3 - 1] * dt;

    double C41 = C[4 - 1][1 - 1] * dt;
    double C42 = C[4 - 1][2 - 1] * dt;
    double C43 = C[4 - 1][3 - 1] * dt;
    double C44 = C[4 - 1][4 - 1] * dt;

    double C51 = C[5 - 1][1 - 1] * dt;
    double C52 = C[5 - 1][2 - 1] * dt;
    double C53 = C[5 - 1][3 - 1] * dt;
    double C54 = C[5 - 1][4 - 1] * dt;
    double C55 = C[5 - 1][5 - 1] * dt;

    var a1 = f(v).times(dt);
    var a2 = f(v.plus(a1.times(C11))).times(dt);
    var a3 = f(v.plus(a1.times(C21)).plus(a2.times(C22))).times(dt);
    var a4 = f(v.plus(a1.times(C31)).plus(a2.times(C32)).plus(a3.times(C33))).times(dt);
    var a5 = f(v.plus(a1.times(C41)).plus(a2.times(C42)).plus(a3.times(C43)).plus(a4.times(C44))).times(dt);
    var a6 = f(v.plus(a1.times(C51)).plus(a2.times(C52)).plus(a3.times(C53)).plus(a4.times(C54)).plus(a5.times(C55)))
        .times(dt);

    var W11 = W[1 - 1][1 - 1];
    var W12 = W[1 - 1][2 - 1];
    var W13 = W[1 - 1][3 - 1];
    var W14 = W[1 - 1][4 - 1];
    var W15 = W[1 - 1][5 - 1];

    var W21 = W[2 - 1][1 - 1];
    var W22 = W[2 - 1][2 - 1];
    var W23 = W[2 - 1][3 - 1];
    var W24 = W[2 - 1][4 - 1];
    var W25 = W[2 - 1][5 - 1];
    var W26 = W[2 - 1][6 - 1];

    var A1 = f(v).plus(a1.times(W11)).plus(a2.times(W12)).plus(a3.times(W13)).plus(a4.times(W14)).plus(a5.times(W15));
    var A2 = f(v).plus(a1.times(W21))
        .plus(a2.times(W22))
        .plus(a3.times(W23))
        .plus(a4.times(W24))
        .plus(a5.times(W25))
        .plus(a6.times(W26));

    var ERR = (1 / dt) * (Math.abs(A1.minus(A2).getNorm()));

    var delta = Math.pow(0.84 * (this.epsilon / ERR), 0.25);

    this.dt = delta * dt;

    if (ERR < epsilon) {
      this.step();
    } else {
      this.VectorStateUpdater.accept(new VectorState(v.plus(A2)));
    }
  }
}