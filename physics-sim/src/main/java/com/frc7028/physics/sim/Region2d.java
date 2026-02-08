package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation2d;

public class Region2d {
  Translation2d min;
  Translation2d max;

  double minX;
  double maxX;

  double minY;
  double maxY;

  public Region2d(Translation2d min, Translation2d max) {
    this.minX = min.getX();
    this.maxX = max.getX();
    this.minY = min.getY();
    this.maxY = max.getY();
  }

  public boolean testPosition(Translation2d position) {
    double X = position.getX();
    double Y = position.getY();
    return minX <= X && X <= maxX && minY <= Y && Y <= maxY;
  }
}
