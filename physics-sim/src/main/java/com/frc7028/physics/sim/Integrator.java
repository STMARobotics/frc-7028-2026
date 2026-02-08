/*
 * file: Integrator.java
 * author: math-rad(github), math.rad elsewhere
 * date: 2/2/2026
 * description: A high precision simulator for solving dv/dt = f(v) employing RK45
 */
package com.frc7028.physics.sim;

import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Function;
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
// Eulers method is "first order" way to solve this differential equation, it is decent, but it does not behave great with sensitive or rapidly changing functions

// We use RK45 to accurate simulate the position, and it also dynamically modulates your time steps to ensure precision if f is very sensitive, and to increase it when smaller steps are not needed.

// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method
// https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf 
// We ignore time here these forces are not in terms of time, just velocity
public class Integrator {
  VectorState vectorState;
  IntegratorResolution integratorResolution;
  Function<forceInput, Translation3d> forceFunction;

  double epsilon;
  int maxIterations;

  public record forceInput(Translation3d position, Translation3d velocity, Rotation3d spin) {
    public forceInput accelerate(Translation3d acceleration, double impulseDuration) {
      return new forceInput(this.position, this.velocity.plus(acceleration.times(impulseDuration)), spin);
    }

    static forceInput from(VectorState state) {
      return new forceInput(state.position, state.velocity, state.spin);
    };
  };

  private static final double[][] C = {
      { 1 / 4d },
      { 3 / 32d, 9 / 32d },
      { 1932 / 2197d, -7200 / 2197d, 7296 / 2197d },
      { 439 / 216d, -8, 3680 / 513d, -845 / 4104d },
      { -8 / 27d, 2, -3544 / 2565d, 1859 / 4104d, -11 / 40d } };

  // K Weights
  private static final double[][] W = {
      { 25 / 216d, 0, 1408 / 2565d, 2197 / 4104d, -1 / 5 },
      { 16 / 135d, 0, 6656 / 12825d, 28561 / 56430d, -9 / 50d, 2 / 55d } };

  public Integrator(
      VectorState vectorState,
      IntegratorResolution integratorResolution,
      Function<forceInput, Translation3d> forceFunction) {
    this.vectorState = vectorState;
    this.integratorResolution = integratorResolution;
    this.forceFunction = forceFunction;

    this.epsilon = integratorResolution.epsilon();
    this.maxIterations = integratorResolution.maxIterations();
  }

  public void reset() {
    this.vectorState.dt = this.integratorResolution.dtInitial();
  }

  private Translation3d f(forceInput input) {
    return this.forceFunction.apply(input);
  }

  public void step() {
    forceInput i = forceInput.from(this.vectorState);
    double err = Double.POSITIVE_INFINITY;

    int iterations = 0;

    // sample each acceleration iteratively, with pre-determined future step scalars for each order

    boolean accepted = false;

    while (!accepted && iterations < this.maxIterations) {
      double dt = this.vectorState.dt;

      Translation3d a1 = f(i);
      Translation3d a2 = f(i.accelerate(a1, dt * C[0][0]));
      Translation3d a3 = f(i.accelerate(a1, dt * C[1][0]).accelerate(a2, dt * C[1][1]));
      Translation3d a4 = f(i.accelerate(a1, dt * C[2][0]).accelerate(a2, dt * C[2][1]).accelerate(a3, dt * C[2][2]));
      Translation3d a5 = f(
          i.accelerate(a1, dt * C[3][0])
              .accelerate(a2, dt * C[3][1])
              .accelerate(a3, dt * C[3][2])
              .accelerate(a4, dt * C[3][3]));
      Translation3d a6 = f(
          i.accelerate(a1, dt * C[4][0])
              .accelerate(a2, dt * C[4][1])
              .accelerate(a3, dt * C[4][2])
              .accelerate(a4, dt * C[4][3])
              .accelerate(a5, dt * C[4][4]));

      Translation3d V1 = i.velocity.plus(a1.times(W[0][0]))
          .plus(a2.times(W[0][1]))
          .plus(a3.times(W[0][2]))
          .plus(a4.times(W[0][3]))
          .plus(a5.times(W[0][4]));

      Translation3d V2 = i.velocity.plus(a1.times(W[1][0]))
          .plus(a2.times(W[1][1]))
          .plus(a3.times(W[1][2]))
          .plus(a4.times(W[1][3]))
          .plus(a5.times(W[1][4]))
          .plus(a6.times(W[1][5]));

      err = abs(V2.getDistance(V1)) / dt;
      System.out.println("err=" + err + " eps=" + this.epsilon + " dt=" + dt);
      if (err <= this.epsilon) {
        this.vectorState.velocity = V2;
        this.vectorState.stepPosition();
        accepted = true;

        double dtScalingFactor = 0.84 * Math.pow((epsilon / Math.max(err, 1e-10)), 0.25);
        this.vectorState.dt = Math.clamp(dt * dtScalingFactor, 1e-6, 1e-2);
        System.out.println("scale=" + dtScalingFactor + " new_dt=" + (dt * dtScalingFactor));
      }

      iterations++;
    }

    // System.out.println(iterations);
  }
}