//
// Created by math-rad on 1/30/26.
// BallisticProjectileState.h
//

#pragma once

#include <Eigen/Core>


class BallisticProjectileState {
public:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angularVelocity;
    double timeElapsed;

    void launch(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& launchVelocity, const Eigen::Vector3d& launchAcceleration, const Eigen::Vector3d& launchAngularVelocity) {
        this->position=initialPosition;
        this->velocity=launchVelocity;
        this->acceleration=launchAcceleration;
        this->angularVelocity=launchAngularVelocity;
        this->timeElapsed=0.0;
    }
};