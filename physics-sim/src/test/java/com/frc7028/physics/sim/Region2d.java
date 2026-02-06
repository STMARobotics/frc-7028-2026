package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation2d;

public class Region2d {
  Translation2d min;
  Translation2d max;

  public Region2d(Translation2d min, Translation2d max) {
    this.min = min;
    this.max = max;
  }

  public boolean testPosition(Translation2d position) {
    Translation2d offset = position.minus(this.min);
    return offset.getX() <= this.max.getX() && offset.getY() <= this.max.getY();
  }
}
