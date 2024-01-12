#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

static void stableQuatToEuler(const Eigen::Quaterniond &quat, Eigen::Vector3d &e)
{
    // q << w,x,y,z
    Eigen::Vector4d q(quat.w(), quat.x(), quat.y(), quat.z());

    double R[3][3];
    R[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    R[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
    R[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    R[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
    R[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    R[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

    // 计算欧拉角
    double sy = sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]);
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2(R[2][1], R[2][2]);
        y = atan2(-R[2][0], sy);
        z = atan2(R[1][0], R[0][0]);
    }
    else
    {
        x = atan2(-R[1][2], R[1][1]);
        y = atan2(-R[2][0], sy);
        z = 0;
    }

    // 将欧拉角转换为连续形式
    if (y < -M_PI / 2 + 1e-6)
    {
        y = -M_PI - y;
        x += M_PI;
        z += M_PI;
    }
    else if (y > M_PI / 2 - 1e-6)
    {
        y = M_PI - y;
        x += M_PI;
        z += M_PI;
    }
    e << x, y, z;
}
