//
// Created by math-rad on 1/30/26.
//

#include "main.h"
#include <iostream>
#include <Eigen/Dense>
#include <numbers>

#include "BallisticSimulator.h"
#include <vector>
#include <Eigen/Geometry>

int main() {
    BallisticSimulator *ballisticSimulator2024 = new BallisticSimulator(
        new BallisticEnvironmentProfile(
            2.0,
            9.8,
            1.225,
            1.0,
            1.0,
            1.0,
            1.0
        ),
        new BallisticSimulatorResolutionProfile(
            0.01,
            0.1,
            0.1,
            0.1,
            0.1,
            std::vector<Eigen::AlignedBox2d> {new Eigen::AlignedBox2d(0, 0)}

        ),
        new BallisticProjectileState(

        )
    );
}