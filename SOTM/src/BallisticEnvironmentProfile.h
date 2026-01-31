//
// Created by math-rad on 1/30/26.
//

#ifndef BALLISTICSIMULATOR_BALLISTICENVIRONMENT_H
#define BALLISTICSIMULATOR_BALLISTICENVIRONMENT_H

#pragma once


class BallisticEnvironmentProfile {
public:
    double ballisticProjectileMass;
    double gravitationalAcceleration;
    double airDensity;
    double ballisticProjectileDragCoefficient;
    double ballisticProjectileCrossSectionalArea;
    double ballisticProjectileLiftCoefficient;
    double ballisticProjectileRadius;

    BallisticEnvironmentProfile(const double ballisticProjectileMass, const double gravitationalAcceleration,
                                const double airDensity,
                                const double ballisticProjectileDragCoefficient,
                                const double ballisticProjectileCrossSectionalArea,
                                const double ballisticProjectileLiftCoefficient, const double ballisticProjectileRadius)
        : ballisticProjectileMass(ballisticProjectileMass),
          gravitationalAcceleration(gravitationalAcceleration),
          airDensity(airDensity), ballisticProjectileDragCoefficient(ballisticProjectileDragCoefficient),
          ballisticProjectileCrossSectionalArea(ballisticProjectileCrossSectionalArea),
          ballisticProjectileLiftCoefficient(ballisticProjectileLiftCoefficient),
          ballisticProjectileRadius(ballisticProjectileRadius) {
    }
};


#endif //BALLISTICSIMULATOR_BALLISTICENVIRONMENT_H
