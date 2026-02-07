package com.frc7028.physics.sim;

import edu.wpi.first.units.measure.Angle;

public record SimulatorResolution(
    double deltaTime,
    Angle deltaAngle,
    double delta_dSpeed,
    double delta_dPosComp,
    double delta_dVelComp,
    double targetMaxDistanceForConvergence,
    int maxIterations) {
}