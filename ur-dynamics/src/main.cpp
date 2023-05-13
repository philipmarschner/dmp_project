#include <iostream>
#include "ur_robot.h"

int main(int, char**) 
{
    std::cout << "Hello, world!\n";

    URRobot robot;

    // double q[] = { 1.0000,   1.0472,    1.0472};
    // double dq[] = { 1.0000,   1.0472,    1.0472};
    // double ddq[] = { 1.0000,   1.0472,    1.0472};

    Eigen::Matrix<double, 6, 1> q, dq, ddq;
    q << 1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472;
    dq << 1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472;
    ddq << 1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472;

    std::cout << "q" << std::endl;
    for (double x : q)
        std::cout << x << ", ";
    std::cout << std::endl;

    std::cout << "dq" << std::endl;
    for (double x : dq)
        std::cout << x << ", ";
    std::cout << std::endl;

    std::cout << "ddq" << std::endl;
    for (double x : ddq)
        std::cout << x << ", ";
    std::cout << std::endl;

    Eigen::Matrix<double, 6, 1> grav = robot.gravity(q);
    std::cout << "gravity vector" << std::endl;
    std::cout << grav << std::endl;

    Eigen::Matrix<double, 6, 6> jac = robot.jacobian(q);
    std::cout << "Jacobian matrix" << std::endl;
    std::cout << jac << std::endl;

    Eigen::Matrix<double, 6, 6> jacDot = robot.jacobianDot(q, dq);
    std::cout << "Jacobian dot matrix" << std::endl;
    std::cout << jacDot << std::endl;

    Eigen::Matrix<double, 6, 6> inertia = robot.inertia(q);
    std::cout << "Inertia matrix" << std::endl;
    std::cout << inertia << std::endl;

    Eigen::Matrix<double, 6, 6> coriolis = robot.coriolis(q, dq);
    std::cout << "Coriolis matrix" << std::endl;
    std::cout << coriolis << std::endl;

    std::cout << "Velocity product" << std::endl;
    std::cout << coriolis * dq << std::endl;

    Eigen::Matrix<double, 6, 1> tau = inertia * ddq + coriolis * dq + grav;
    std::cout << "Torque" << std::endl;
    std::cout << tau << std::endl;

}

