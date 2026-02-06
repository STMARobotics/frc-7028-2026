package frc.ballisticSimulator;

import edu.wpi.first.math.geometry.Translation3d;

public class Region3d {
  Translation3d min;
  Translation3d max;

  public Region3d(Translation3d min, Translation3d max) {
    this.min = min;
    this.max = max;
  }

  public boolean testPosition(Translation3d position) {
    Translation3d offset = position.minus(this.min);
    return offset.getX() <= this.max.getX() && offset.getY() <= this.max.getY() && offset.getZ() <= this.max.getZ();
  }
}