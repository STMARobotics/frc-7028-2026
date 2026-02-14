package com.frc7028.physics.sim;

import java.util.ArrayList;

public record FieldMetrics(
    Region2d fieldPlane,
    Region3d fieldBounds,
    ArrayList<Region3d> obstacles,
    ArrayList<Region3d> rejectRegions,
    Region3d targetRegion,
    double startingHeight) {
}
