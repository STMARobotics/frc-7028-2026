package com.frc7028.physics.sim;

public class BallisticEnvironmentProfile {
  public double ballisticProjectileMass;
  public double gravitationalAcceleration;
  public double airDensity;
  public double ballisticProjectileDragCoefficient;
  public double ballisticProjectileCrossSectionalArea;
  public double ballisticProjectileMagnusCoefficient;
  public double ballisticProjectileRadius;

  public BallisticEnvironmentProfile(
      double ballisticProjectileMass,
      double gravitationalAcceleration,
      double airDensity,
      double ballisticProjectileDragCoefficient,
      double ballisticProjectileCrossSectionalArea,
      double ballisticProjectileMagnusCoefficient,
      double ballisticProjectileRadius) {
    this.ballisticProjectileMass = ballisticProjectileMass;
    this.gravitationalAcceleration = gravitationalAcceleration;
    this.airDensity = airDensity;
    this.ballisticProjectileDragCoefficient = ballisticProjectileDragCoefficient;
    this.ballisticProjectileMagnusCoefficient = ballisticProjectileMagnusCoefficient;
    this.ballisticProjectileRadius = ballisticProjectileRadius;
    this.ballisticProjectileCrossSectionalArea = Math.PI * Math.pow(ballisticProjectileRadius, 2);
  }

}