package frc.ballisticSimulator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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
  private static final double[][] KCOEF = { 
    { 0 },
    { 1/4 },
    { 3 / 32, 9 / 32},
    { 1932 / 2197, -7200 / 2197, 7296 / 2197},
    { 439 / 216, -8, 3680 / 513, -845 / 4104},
    { 2, -3544 / 2565, 1859 / 4104, -11 / 40}
   };

   // K Weights
   private static final double[][] KW = {
    { 25 / 216, 0, 1408 / 2565, 2197 / 4104 -1/5},
    { 16 / 135, 0, 6656 / 12825, 28561 / 56430, 9 / 50, 2 / 55 }
   };

// spotless:on

  private record VectorState(Translation3d Velocity, Rotation3d Spin) {
    public peek() {

    }
  }

  public Integrator() {

  }
}