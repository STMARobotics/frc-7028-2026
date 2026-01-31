//
// Created by math-rad on 1/30/26.
//

#include <iostream>
#include "BallisticProjectileState.h"
#include "BallisticEnvironmentProfile.h"
#include "BallisticSimulatorResolutionProfile.h"

#include <Eigen/Dense>


class BallisticSimulator {
public:
    BallisticEnvironmentProfile environmentProfile;
    BallisticSimulatorResolutionProfile resolutionProfile;
    BallisticProjectileState projectileState;

    BallisticSimulator(const BallisticEnvironmentProfile &environmentProfile, const BallisticSimulatorResolutionProfile &resolutionProfile,
              const BallisticProjectileState &projectileState) :

        environmentProfile(environmentProfile), resolutionProfile(resolutionProfile), projectileState(projectileState) {

    }

    BallisticSimulator(BallisticEnvironmentProfile * ballistic_environment_profile, BallisticSimulatorResolutionProfile * ballistic_simulator_resolution_profile, BallisticProjectileState * ballistic_projectile_state);
};
