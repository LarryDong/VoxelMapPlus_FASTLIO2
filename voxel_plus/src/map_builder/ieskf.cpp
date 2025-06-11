#include "ieskf.h"

extern vector<Eigen::Vector3d> g_p2v_, g_p2plane_;
extern int g_scan_cnt;

extern double g_slam_init_time, g_ros_running_time;



namespace kf
{

    Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &inp)
    {
        return Sophus::SO3d::leftJacobian(inp).transpose();
    }

    void State::operator+=(const Vector23d &delta)
    {
        pos += delta.segment<3>(0);
        rot *= Sophus::SO3d::exp(delta.segment<3>(3)).matrix();
        rot_ext *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
        pos_ext += delta.segment<3>(9);
        vel += delta.segment<3>(12);
        bg += delta.segment<3>(15);
        ba += delta.segment<3>(18);
        g = Sophus::SO3d::exp(getBx() * delta.segment<2>(21)).matrix() * g;
    }

    void State::operator+=(const Vector24d &delta)
    {
        pos += delta.segment<3>(0);
        rot *= Sophus::SO3d::exp(delta.segment<3>(3)).matrix();
        rot_ext *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
        pos_ext += delta.segment<3>(9);
        vel += delta.segment<3>(12);
        bg += delta.segment<3>(15);
        ba += delta.segment<3>(18);
        g = Sophus::SO3d::exp(delta.segment<3>(21)).matrix() * g;
    }

    Vector23d State::operator-(const State &other)
    {
        Vector23d delta = Vector23d::Zero();
        delta.segment<3>(0) = pos - other.pos;
        delta.segment<3>(3) = Sophus::SO3d(other.rot.transpose() * rot).log();
        delta.segment<3>(6) = Sophus::SO3d(other.rot_ext.transpose() * rot_ext).log();
        delta.segment<3>(9) = pos_ext - other.pos_ext;
        delta.segment<3>(12) = vel - other.vel;
        delta.segment<3>(15) = bg - other.bg;
        delta.segment<3>(18) = ba - other.ba;

        double v_sin = (Sophus::SO3d::hat(g) * other.g).norm();
        double v_cos = g.transpose() * other.g;
        double theta = std::atan2(v_sin, v_cos);
        Eigen::Vector2d res;
        if (v_sin < 1e-11)
        {
            if (std::fabs(theta) > 1e-11)
            {
                res << 3.1415926, 0;
            }
            else
            {
                res << 0, 0;
            }
        }
        else
        {
            res = theta / v_sin * other.getBx().transpose() * Sophus::SO3d::hat(other.g) * g;
        }
        delta.segment<2>(21) = res;
        return delta;
    }

    Matrix3x2d State::getBx() const
    {
        Matrix3x2d res;
        res << -g[1], -g[2],
            GRAVITY - g[1] * g[1] / (GRAVITY + g[0]), -g[2] * g[1] / (GRAVITY + g[0]),
            -g[2] * g[1] / (GRAVITY + g[0]), GRAVITY - g[2] * g[2] / (GRAVITY + g[0]);
        res /= GRAVITY;
        return res;
    }

    Matrix3x2d State::getMx() const
    {

        return -Sophus::SO3d::hat(g) * getBx();
    }

    Matrix3x2d State::getMx(const Eigen::Vector2d &res) const
    {
        Matrix3x2d bx = getBx();
        Eigen::Vector3d bu = bx * res;
        return -Sophus::SO3d::exp(bu).matrix() * Sophus::SO3d::hat(g) * Sophus::SO3d::leftJacobian(bu).transpose() * bx;
    }

    Matrix2x3d State::getNx() const
    {
        return 1 / GRAVITY / GRAVITY * getBx().transpose() * Sophus::SO3d::hat(g);
    }

    IESKF::IESKF() = default;

    int IESKF::P_ID = 0, IESKF::R_ID = 3, IESKF::ER_ID = 6, IESKF::EP_ID = 9, IESKF::V_ID = 12, IESKF::BG_ID = 15, IESKF::BA_ID = 18, IESKF::G_ID = 21;

    void IESKF::predict(const Input &inp, double dt, const Matrix12d &Q)
    {
        Vector24d delta = Vector24d::Zero();
        delta.segment<3>(0) = x_.vel * dt;
        delta.segment<3>(3) = (inp.gyro - x_.bg) * dt;
        delta.segment<3>(12) = (x_.rot * (inp.acc - x_.ba) + x_.g) * dt;
        F_.setIdentity();
        F_.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity() * dt;
        F_.block<3, 3>(3, 3) = Sophus::SO3d::exp(-(inp.gyro - x_.bg) * dt).matrix();
        F_.block<3, 3>(3, 15) = -rightJacobian((inp.gyro - x_.bg) * dt) * dt;
        F_.block<3, 3>(12, 3) = -x_.rot * Sophus::SO3d::hat(inp.acc - x_.ba) * dt;
        F_.block<3, 3>(12, 18) = -x_.rot * dt;
        F_.block<3, 2>(12, 21) = x_.getMx() * dt;
        F_.block<2, 2>(21, 21) = x_.getNx() * x_.getMx();

        G_.setZero();
        G_.block<3, 3>(3, 0) = -rightJacobian((inp.gyro - x_.bg) * dt) * dt;
        G_.block<3, 3>(12, 3) = -x_.rot * dt;
        G_.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
        G_.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;
        x_ += delta;
        P_ = F_ * P_ * F_.transpose() + G_ * Q * G_.transpose();
    }


    // TODO: Rewrite the update function!!!

    void IESKF::update(bool use_p2v, ScanRegisterViewer& my_viewer){
    // void IESKF::update(bool use_p2v, std::vector<Eigen::Vector3d>& p2plane, std::vector<Eigen::Vector3d>& p2v){
        //~ x_是状态量，在函数中更新了x_以及对应的P_；
        const State predict_x = x_;         //~ predict_x is never changed.
        Matrix23d P_input = P_;
        Matrix23d P_ori, P_new;

        SharedState shared_data;
        shared_data.iter_num = 0;
        Vector23d delta = Vector23d::Zero();        //~ ATTENTION: `delta` is vector23, `x_` is a 'State'
        Matrix23d L = Matrix23d::Identity();
        State new_x, original_x;

        double debug_offset_x = 0.00;
        double debug_offset_y = 0.00;

        cout << "\n-------------------------\n" << endl;

        ///////////////////////////////////////////////////  Original Method  ///////////////////////////////////////////////////
        if(!use_p2v){
            for (size_t i = 0; i < max_iter_; i++)
            {
                // update viewer on first iteration.

                if(i==0){
                    // change x to increate.
                    x_.pos[0] += debug_offset_x;           // add an offset for better visualization
                    x_.pos[1] += debug_offset_y;
                }

                // func_ 动态绑定了： sharedUpdateFunc
                // sharedUpdateFunc calculate each `H`, and `b`.
                func_(x_, shared_data, my_viewer, i==0);                 

                H_.setZero();
                b_.setZero();
                delta = x_ - predict_x;                     // `delta` is vector, `x_` is 'State', "operation-" is defined.
                Matrix23d J = Matrix23d::Identity();
                J.block<3, 3>(3, 3) = rightJacobian(delta.segment<3>(3));                       // rotation's R-jacobian
                J.block<3, 3>(6, 6) = rightJacobian(delta.segment<3>(6));                       // ext-rot's R-jacobian
                J.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));     // graivity part.

                // 这部分是先验的线性化
                // x^ = argmin { (x-pred_x)*P^{-1}*(x-pred_x) + \sum |r_i|_R }，变成 Hx=b 后，H/b 第一部分是先验的偏导，第二部分是 shared_data 的 H和b
                // 所以形式上，采用的是 Gauss-Newton 的方式优化的MAP，并不是信息矩阵形势下的Kalman
                b_ += (J.transpose() * P_.inverse() * delta);       //~ ? Why +=, not assign?
                H_ += (J.transpose() * P_.inverse() * J);

                cout << "\n Original Update. Time: " << g_ros_running_time - g_slam_init_time << ", Iteration: " << i << endl;

                cout << "\n delta.segment<3>(3): \n" << delta.segment<3>(3) << endl;

                cout <<" \n Test Jacobian: " << endl;
                cout << "\n delta 3 R-jacobian: \n" << rightJacobian(delta.segment<3>(6));
                cout << "\n delta 6 R-jacobian: \n" << rightJacobian(delta.segment<3>(3));
                cout << "\n J: \n" << J << endl;

                cout << "\n\n Ori: 11111111111 " << endl;
                cout << "\n H: \n" << H_ << endl;
                cout << "\n b: \n" << b_ << endl;
                cout << "\n delta rot: " << delta[0] << ", " << delta[1] << ", " << delta[2] << endl;
                cout << "\n delta pos: " << delta[3] << ", " << delta[4] << ", " << delta[5] << endl;

                H_.block<12, 12>(0, 0) += shared_data.H;
                b_.block<12, 1>(0, 0) += shared_data.b;
                delta = -H_.inverse() * b_;
                x_ += delta;
                shared_data.iter_num += 1;

                cout << "\n\n Ori: 222222222222 " << endl;
                cout << "\n H: \n" << H_ << endl;
                cout << "\n b: \n" << b_ << endl;
                cout << "\n delta rot: " << delta[0] << ", " << delta[1] << ", " << delta[2] << endl;
                cout << "\n delta pos: " << delta[3] << ", " << delta[4] << ", " << delta[5] << endl;



                // cout << "H: \n" << H_ << endl;
                // cout << "\n b: \n" << b_ << endl;
                // cout << "\n J: \n" << J << endl;
                // cout << "\n delta: \n" << delta << endl;
                // cout << "\n rot: \n" << x_.rot << " \n ,  pos: " << x_.pos.transpose() << endl;

                // // DEBUG:
                // cout << "\n Original Update. Time: " << g_ros_running_time - g_slam_init_time << ", Iteration: " << i << endl;
                // cout << "delta rot: " << delta[0] << ", " << delta[1] << ", " << delta[2] << endl;
                // cout << "delta pos: " << delta[3] << ", " << delta[4] << ", " << delta[5] << endl;
                // cout << "x pos: " << x_.pos[0] << ", " << x_.pos[1] << ", " << x_.pos[2] << endl;
                // cout << "H: \n" << H_ << endl;
                // cout << "\n b: \n" << b_ << endl;

                if (delta.maxCoeff() < eps_)
                    break;
            }
            //~ Kalman Filter Updata
            L.block<3, 3>(3, 3) = rightJacobian(delta.segment<3>(3));
            L.block<3, 3>(6, 6) = rightJacobian(delta.segment<3>(6));
            L.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));     // gravity-term, dim=2 (length=Gravity)
            P_ = L * H_.inverse() * L.transpose();

            // print result.
            P_ori = P_;
            original_x = x_;
        }
        // ///////////////////////////////////////////////////  Original Method  ///////////////////////////////////////////////////

        
        ///////////////////////////////////////////////////  NEW P2V Method  ///////////////////////////////////////////////////
        // if(use_p2v){
            ROS_WARN_ONCE("==> Now Using P2V Update!");

            // Reset values
            x_ = predict_x;         // reset back to inital value.      TODO: REMOVE THIS!!! (Not necessary)
            delta = Vector23d::Zero();
            L = Matrix23d::Identity();
            P_ = P_input;

            SharedState shared_data_p2v;
            shared_data_p2v.iter_num = 0;
            for (size_t i = 0; i < max_iter_; i++)
            {
                if(i==0){
                    // change x to increate.
                    x_.pos[0] += debug_offset_x;           // add an offset for better visualization
                    x_.pos[1] += debug_offset_y;
                }
                // cout << "[func_p2v_] begin. Iteration: " << i <<"/" << max_iter_ << endl;

                func_p2v_(x_, shared_data_p2v, my_viewer, i==0);                 // func_p2v_ 动态绑定了： sharedUpdateFunc_p2v
                
                // cout << "[func_p2v_] end." << endl;

                H_.setZero();
                b_.setZero();
                delta = x_ - predict_x;


                // ISSUE: `H_` is always increasing!!!!!!, `b_` scale is very different from original b

                Matrix23d J = Matrix23d::Identity();
                J.block<3, 3>(3, 3) = rightJacobian(delta.segment<3>(3));
                J.block<3, 3>(6, 6) = rightJacobian(delta.segment<3>(6));
                J.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
                b_ += (J.transpose() * P_.inverse() * delta);
                H_ += (J.transpose() * P_.inverse() * J);


                cout << "\n P2V Update. Time: " << g_ros_running_time - g_slam_init_time << ", Iteration: " << i << endl;

                cout << "\n\n P2V: 11111111111 " << endl;
                cout << "\n H: \n" << H_ << endl;
                cout << "\n b: \n" << b_ << endl;
                cout << "\n delta rot: " << delta[0] << ", " << delta[1] << ", " << delta[2] << endl;
                cout << "\n delta pos: " << delta[3] << ", " << delta[4] << ", " << delta[5] << endl;


                H_.block<12, 12>(0, 0) += shared_data_p2v.H;
                b_.block<12, 1>(0, 0) += shared_data_p2v.b;




                delta = -H_.inverse() * b_;
                x_ += delta;
                shared_data_p2v.iter_num += 1;

                cout << "\n\n P2V: 222222222222 " << endl;
                cout << "\n H: \n" << H_ << endl;
                cout << "\n b: \n" << b_ << endl;
                cout << "\n delta rot: " << delta[0] << ", " << delta[1] << ", " << delta[2] << endl;
                cout << "\n delta pos: " << delta[3] << ", " << delta[4] << ", " << delta[5] << endl;


                // cout << "H: \n" << H_ << endl;
                // cout << "\n b: \n" << b_ << endl;
                // cout << "\n J: \n" << J << endl;
                // cout << "\n delta: \n" << delta << endl;
                // cout << "\n rot: \n" << x_.rot << " \n ,  pos: " << x_.pos.transpose() << endl;

                // cout << "\n P2V Update. Time: " << g_ros_running_time - g_slam_init_time << ", Iteration: " << i << endl;
                // cout << "delta rot: " << delta[0] << ", " << delta[1] << ", " << delta[2] << endl;
                // cout << "delta pos: " << delta[3] << ", " << delta[4] << ", " << delta[5] << endl;
                // cout << "x pos: " << x_.pos[0] << ", " << x_.pos[1] << ", " << x_.pos[2] << endl;

                // cout << "H: \n" << H_ << endl;
                // cout << "\n b: \n" << b_ << endl;

                if (delta.maxCoeff() < eps_)
                    break;
            }
            L.block<3, 3>(3, 3) = rightJacobian(delta.segment<3>(3));
            L.block<3, 3>(6, 6) = rightJacobian(delta.segment<3>(6));
            L.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
            P_ = L * H_.inverse() * L.transpose();
            ///////////////////////////////////////////////////  NEW P2V Method  ///////////////////////////////////////////////////

            P_new = P_;
            new_x = x_;
        // }
        // new_x.printInfo("------------- [P2V] State Estimation. -------------");
        

        // Select which state to use, based on init.
        if (use_p2v){
            x_ = new_x;
            P_ = P_new;
        }
        else{
            x_ = original_x;
            P_ = P_ori;
        }

    }

} // namespace kf