#ifndef ESKF_UTIL_HPP
#define ESKF_UTIL_HPP

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

Eigen::Vector3f toEuler(const Eigen::Quaternionf q);
Eigen::Matrix4f toSE3(const Eigen::Quaternionf q, const Eigen::Vector3f t);
Eigen::Matrix3f toSkew(const Eigen::Vector3f in);
Eigen::Quaternionf toQuaternion(const Eigen::Vector3f euler);
Eigen::Quaternionf toQuat(const Eigen::Vector3f in);
Eigen::MatrixXf getQdtheta(const Eigen::Quaternionf q);

#endif
