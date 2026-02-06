package frc.ballisticSimulator;

class BallisticEnvironmentProfile {
  double ballisticProjectileMass;
  double gravitationalAcceleration;
  double airDensity;
  double ballisticProjectileDragCoefficient;
  double ballisticProjectileCrossSectionalArea;
  double ballisticProjectileMagnusCoefficient;
  double ballisticProjectileRadius;

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