//
// Created by math-rad on 1/30/26.
//

#include "main.h"
#include <iostream>
#include <Eigen/Dense>
#include <numbers>

using std::sin;
using std::cos;
using std::tan;
using std::sqrt;
using std::floor;
using std::atan2;

inline double sqr(const double x) {
    return x * x;
}

int main() {
    std::cout << sqr(std::numbers::pi_v<double>) << std::endl;
    return 0;
}