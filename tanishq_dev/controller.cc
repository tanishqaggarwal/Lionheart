#include "tanishq_dev/controller.h"
#include "quaternion.h"

namespace Lionheart {

template <typename T>
AttitudeRegulationControllerT<T>::AttitudeRegulationControllerT(T kp, T kd)
    : kp(kp), kd(kd) {}

template <typename T>
Vector3<T>
AttitudeRegulationControllerT<T>::update(const Matrix3<T> &attitude_sns,
                                         const Matrix3<T> &attitude_cmd,
                                         const Vector3<T> &angvel_sns) {

    const Quat<T> sns = to_quat(attitude_sns);
    const Quat<T> cmd = to_quat(attitude_cmd);

    const Quat<T> err = cmd.product(sns);
    const Vector<T> p{err.x, err.y, err.z};

    /// 7.7 from Fundamentals of Spacecraft Attitude Determination and Control
    /// by F.Landis Markley, John L. Crassidis.
    return -1 * kp * p - kd * angvel_sns;
}

} // namespace Lionheart