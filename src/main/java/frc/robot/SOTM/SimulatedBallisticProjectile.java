package frc.robot.SOTOM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Mass;

public class SimulatedBallisticProjectile {
  public Mass mass;
  public double volume;
  public double surfaceArea;

  public Translation3d simulatedPosition;

  public SimulatedBallisticProjectile(Mass mass, double volume, double surfaceArea) {
    this.mass = mass;
    this.volume = volume;
    this.surfaceArea = surfaceArea;
  }
}
