#include "tanishq_dev/linalg.h"

namespace Lionheart
{
template<typename T>
struct AttitudeRegulationControllerT
{
    AttitudeRegulationControllerT(T kp, T kd);
    Vector3<T> update(
        const Matrix3<T>& attitude_sns,
        const Matrix3<T>& attitude_cmd,
        const Vector3<T>& angvel_sns);

    T kp;
    T kd;
};
}