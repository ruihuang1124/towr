//
// Created by ray on 9/25/24.
//

#ifndef TOWR_ROS_MINI_CHEETAH_LEG_INVERSE_KINEMATICS_H
#define TOWR_ROS_MINI_CHEETAH_LEG_INVERSE_KINEMATICS_H

#include <Eigen/Dense>

namespace towr {

    enum MiniCheetahJointID
    {
        HAA = 0, HFE, KFE, ArcdoglegJointCount
    };

    class MiniCheetahlegInverseKinematics
    {
    public:
        using Vector3d = Eigen::Vector3d;
        enum KneeBend
        {
            Forward, Backward
        };

        /**
         * @brief Default c'tor initializing leg lengths with standard values.
         */
        MiniCheetahlegInverseKinematics() = default;
        virtual ~MiniCheetahlegInverseKinematics() = default;

        /**
         * @brief Returns the joint angles to reach a Cartesian foot position.
         * @param ee_pos_H  hip-aa (H) 坐标系中脚的位置 xyz
         */
        Vector3d GetJointAngles(const Vector3d &ee_pos_H, int _sideSign) const;

        /**
         * @brief Restricts the joint angles to lie inside the feasible range
         * @param q[in/out]  Current joint angle that is adapted if it exceeds
         * the specified range.
         * @param joint  Which joint (HAA, HFE, KFE) this value represents.
         */
        void EnforceLimits(double &q, MiniCheetahJointID joint) const;

    private:
        double _abadLinkLength = 0.062;
        double _hipLinkLength = 0.209;
        double _kneeLinkLength = 0.195;
        static double q1_ik(double py, double pz, double l1);
        static double q3_ik(double b3z, double b4z, double b);
        static double q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z);
    };
}

#endif //TOWR_ROS_MINI_CHEETAH_LEG_INVERSE_KINEMATICS_H
