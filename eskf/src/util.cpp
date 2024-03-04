#include <eskf/util.hpp>

Eigen::Vector3f toEuler(const Eigen::Quaternionf q){
    Eigen::Vector3f euler_zyx = q.matrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
    Eigen::Vector3f euler_xyz(euler_zyx(2), euler_zyx(1), euler_zyx(0)); // roll, pitch, yaw
    return euler_xyz;
}

Eigen::Matrix4f toSE3(const Eigen::Quaternionf q, const Eigen::Vector3f t)
{
    Eigen::Matrix4f H(Eigen::Matrix4f::Identity());
    Eigen::Matrix3f R(q.matrix());
    Eigen::Vector3f T(t);

    H.block<3, 3>(0, 0) = R;
    H.block<3, 1>(0, 3) = T;

    return H;
}

Eigen::Matrix3f toSkew(const Eigen::Vector3f in)
{
    Eigen::Matrix3f out;

    out << 0, -in(2), in(1),
        in(2), 0, -in(0),
        -in(1), in(0), 0;

    return out;
}

Eigen::Quaternionf toQuaternion(const Eigen::Vector3f euler)
{
    Eigen::AngleAxisf roll(euler(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch(euler(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw(euler(2), Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q(yaw * pitch * roll);

    return q;
}

Eigen::Quaternionf toQuat(const Eigen::Vector3f in) {
    // Calculate the angle of rotation from the magnitude of the input vector
    float angle = in.norm();

    // Define the rotation axis; if the angle is zero (no rotation), use a default axis (here, X-axis)
    Eigen::Vector3f axis = (angle == 0) ? Eigen::Vector3f(1, 0, 0) : in.normalized();

    // Create a quaternion representing the rotation using Eigen's AngleAxis constructor
    return Eigen::Quaternionf(Eigen::AngleAxisf(angle, axis));
}

Eigen::Vector4f quat_to_hamilton(const Eigen::Quaternionf q)
{
    return (Eigen::Vector4f() << q.coeffs().block<1, 1>(3, 0), // w
            q.coeffs().block<3, 1>(0, 0))
        .finished(); // x, y, z
}

Eigen::Quaternionf quat_from_hamilton(const Eigen::Vector4f q)
{
    return Eigen::Quaternionf(
        (Eigen::Vector4f() << q.block<3, 1>(1, 0), // x, y, z
         q.block<1, 1>(0, 0)                       // w
         )
            .finished());
}

Eigen::MatrixXf getQdtheta(const Eigen::Quaternionf q){
    Eigen::Matrix<float, 4, 3> Q_dtheta;

    Q_dtheta << -q.x(), -q.y(), -q.z(),
                q.w(), -q.z(), q.y(),
                q.z(), q.w(), -q.x(),
                -q.y(), q.x(), q.w();
    
    Q_dtheta *= 0.5;

    return Q_dtheta;
}