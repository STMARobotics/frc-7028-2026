package frc.ballisticSimulator;

class BallisticEnvironmentProfile {
  double ballisticProjectileMass;
  double gravitationalAcceleration;
  double airDensity;
  double ballisticProjectileDragCoefficient;
  double ballisticProjectileCrossSectionalArea;
  double ballisticProjectileLiftCoefficient;
  double ballisticProjectileRadius;

  public BallisticEnvironmentProfile(
      double ballisticProjectileMass,
      double gravitationalAcceleration,
      double airDensity,
      double ballisticProjectileDragCoefficient,
      double ballisticProjectileCrossSectionalArea,
      double ballisticProjectileLiftCoefficient,
      double ballisticProjectileRadius) {
    this.ballisticProjectileMass = ballisticProjectileMass;
    this.gravitationalAcceleration = gravitationalAcceleration;
    this.airDensity = airDensity;
    this.ballisticProjectileDragCoefficient = ballisticProjectileDragCoefficient;
    this.ballisticProjectileCrossSectionalArea = ballisticProjectileCrossSectionalArea;
    this.ballisticProjectileLiftCoefficient = ballisticProjectileLiftCoefficient;
    this.ballisticProjectileRadius = ballisticProjectileRadius;
  }

}