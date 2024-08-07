#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <Eigen/Core>

// 自定义点类型定义
namespace ouster_ros {
namespace OS1 {
struct EIGEN_ALIGN16 PointOS1 {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline PointOS1 make(float x, float y, float z, float intensity,
                                uint32_t t, uint16_t reflectivity, uint8_t ring,
                                uint16_t noise, uint32_t range) {
        return {x, y, z, 0.0, intensity, t, reflectivity, ring, noise, range};
    }
};
}  // namespace OS1
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointOS1,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, t, t)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (uint16_t, noise, noise)
    (uint32_t, range, range)
)
