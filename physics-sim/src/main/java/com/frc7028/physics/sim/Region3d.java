package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation3d;

public class Region3d {
  double minX;
  double minY;
  double minZ;

  double maxX;
  double maxY;
  double maxZ;
  public Translation3d center;

  public Region3d(Translation3d min, Translation3d max) {
    this.minX = min.getX();
    this.minY = min.getY();
    this.minZ = min.getZ();
    this.maxX = max.getX();
    this.maxY = max.getY();
    this.maxZ = max.getZ();

    this.center = min.plus(max.minus(min).times(0.5));
  }

  public boolean testPosition(Translation3d position) {
    double X = position.getX();
    double Y = position.getY();
    double Z = position.getZ();

    return minX <= X && X <= maxX && minY <= Y && Y <= maxY && minZ <= Z && Z <= maxZ;
  }
}