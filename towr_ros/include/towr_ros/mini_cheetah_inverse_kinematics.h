//
// Created by ray on 9/25/24.
//

#ifndef TOWR_ROS_MINI_CHEETAH_INVERSE_KINEMATICS_H
#define TOWR_ROS_MINI_CHEETAH_INVERSE_KINEMATICS_H

#include <xpp_vis/inverse_kinematics.h>
#include <towr_ros/mini_cheetah_leg_inverse_kinematics.h>

using namespace xpp;

namespace towr
{

/**
 * @brief Inverse kinematics function for the cyberdog robot.
 */
class MiniCheetahInverseKinematics : public InverseKinematics
    {
    public:
        MiniCheetahInverseKinematics() = default;
        virtual ~MiniCheetahInverseKinematics() = default;

        /**
         * @brief Returns joint angles to reach for a specific foot position.
         * @param pos_B  3D-position of the foot expressed in the base frame (B).
         */
        Joints GetAllJointAngles(const EndeffectorsPos &pos_b) const override;

        /**
         * @brief Number of endeffectors (feet, hands) this implementation expects.
         */
        int GetEECount() const override { return 4; };

    private:
        Vector3d base2hip_LF_ = Vector3d(0.19, 0.049, 0.0);
        MiniCheetahlegInverseKinematics leg;
    };

} /* namespace towr */

#endif //TOWR_ROS_MINI_CHEETAH_INVERSE_KINEMATICS_H
