# FRC Ballistic Trajectory Lookup table (WIP)

Designed to compute a look-up table that can handle shoot-on-the-move for ballistic aiming. 

I used RKF45 to simulate projectile motion so that it can accurately account for drag, magnus, and any other forces
 - I made it capable of seamless force integration
  - simply provide a force function given some state parameters to "integrate" over
  - you are essentialy solving an ODE of the form:
    - `dv/dt = f(v, omega)` ~ drag + magnus + gravity

For RKF45 I referenced
 - https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method (table, info)
 - https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf (implementation though)

I chose RKF45 because I wanted to optimize accuracy but also time through its adaptive time stepping. 

To "find" our shot parameters(yaw pitch speed), we can treat our inputs and outputs as vectors, or a matrix as done in the code's implementation. 

Define an error vector function, S(simulation):
 - S(<`yaw`, `pitch`, `speed`>) = <`e_x`, `e_y`, `e_z`> 
  - where `e` represents an error vector that is the displacement from the nearest point on the trajectory to the target
  - One full simulation can be mapped to this very input/output format, and it behaves well since it can compose a square jacobian

We can then use the the [multi-variable Newton-Raphson Method](https://pages.hmc.edu/ruye/MachineLearning/lectures/ch2/node7.html) to "find", or converge, the optimal parameters for some given conditions. The computation is relatively unsophisticated, just take the jacobian, invert it, and multiply that to your initial error vector(or matrix) to get a corrective step. The actualization of the end-goal(to select a working shot), however, is not trivial, as you must select optimal error metrics. For example, you must consider the conditions under which your standard newton method operates, i.e, a well-behaved curve or surface that is continuous. Simply choosing the end position-to-target displacement as your error vector will yield unstable metrics since these can "jump" suddenly upon collisions in the simulation, which creates discontinuities. Previously, as seen in the whiteboard attachment, I used error in yaw, pitch, and height. Currently, I've arrived as using the point in your trajectory that is the closest to the target position, this should theoretically yield mostly continuous error vectors over variations in our input parameters(yaw pitch speed), as collisions shouldn't as often propagate these discontinuities as seen in the simple end-position-to-target displacement vector. 