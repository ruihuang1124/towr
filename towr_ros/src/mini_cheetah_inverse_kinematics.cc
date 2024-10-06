#include <towr_ros/mini_cheetah_inverse_kinematics.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

namespace towr
{

    Joints
    MiniCheetahInverseKinematics::GetAllJointAngles(const EndeffectorsPos &x_B) const
    {
        Vector3d ee_pos_B; // Base坐标系下的落足点
        Vector3d ee_pos_H; // Hip坐标系下的落足点
        std::vector <Eigen::VectorXd> q_vec;

        // make sure always exactly 4 elements
        auto pos_B = x_B.ToImpl(); // Base坐标系下的落足点
        pos_B.resize(4, pos_B.front());

        Vector3d _pHip2B;

        int _sideSign;

        for(int ee = 0; ee < pos_B.size(); ++ee)
        {

            using namespace quad;
            switch(ee)
            {
                case RF:
                    ee_pos_B = pos_B.at(ee);
                    _pHip2B << 0.19, -0.049, 0;
                    _sideSign = -1;
                    break;
                case LF:
                    ee_pos_B = pos_B.at(ee);
                    _pHip2B << 0.19, 0.049, 0;
                    _sideSign = 1;
                    break;
                case RH:
                    ee_pos_B = pos_B.at(ee);
                    _pHip2B << -0.19, -0.049, 0;
                    _sideSign = -1;
                    break;
                case LH:
                    ee_pos_B = pos_B.at(ee);
                    _pHip2B << -0.19, 0.049, 0;
                    _sideSign = 1;
                    break;
                default: // joint angles for this foot do not exist
                    break;
            }

//			ee_pos_H = ee_pos_B - base2hip_LF_;
            ee_pos_H = ee_pos_B - _pHip2B;
            q_vec.push_back(leg.GetJointAngles(ee_pos_H, _sideSign));
        }

        return Joints(q_vec);
    }

} /* namespace xpp */