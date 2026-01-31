//
// Created by math-rad on 1/30/26.
//

#pragma once
#include <Eigen/Geometry>
#include <vector>

class BallisticSimulatorResolutionProfile {
public:
    double deltaTime;
    double deltaAngle;
    double deltaSpeed;
    double deltaPositionalComponent;
    double deltaVelocityComponent;

    std::vector<Eigen::AlignedBox2d> fieldRegions;
    Eigen::AlignedBox3d fieldBounds;
    std::vector<Eigen::AlignedBox3d> obstacleRegions;
    std::vector<Eigen::AlignedBox3d> rejectRegions;

    BallisticSimulatorResolutionProfile(const double &deltaTime, const double &deltaAngle, const double &deltaSpeed,
                                        const double &deltaPositionalComponent,
                                        const double &deltaVelocityComponent,
                                        const std::vector<Eigen::AlignedBox2d> &fieldRegions,
                                        const Eigen::AlignedBox3d &fieldBounds,
                                        const std::vector<Eigen::AlignedBox3d> &obstacleRegions,
                                        const std::vector<Eigen::AlignedBox3d> &rejectRegions


    )
    :
    deltaTime (deltaTime),
            deltaAngle(deltaAngle),
            deltaSpeed(deltaSpeed),
            deltaPositionalComponent(
                deltaPositionalComponent),
            deltaVelocityComponent(
                deltaVelocityComponent),
            fieldRegions(fieldRegions),
            fieldBounds(fieldBounds),
            obstacleRegions(obstacleRegions),
            rejectRegions(rejectRegions) {
    }
};
