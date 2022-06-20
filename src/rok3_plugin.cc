/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;
// VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol, std::string leg = "L");

Vector3d r_des;
MatrixXd C_des;
VectorXd tar_q_left(6),q(6);

VectorXd q_global(6);
Vector3d init_pos;
Vector3d cmd_pos, cur_pos;
VectorXd q_left_cmd(6), r_left_des(6);
MatrixXd C_left_des(3,3);

MatrixXd goal_left_rot(3,3);
MatrixXd init_left_rot(3,3);
MatrixXd current_left_rot(3,3);
MatrixXd cmd_left_rot(3,3);

MatrixXd goal_right_rot(3,3);
MatrixXd init_right_rot(3,3);
MatrixXd current_right_rot(3,3);
MatrixXd cmd_right_rot(3,3);

VectorXd q_init_L(6);
VectorXd q_init_R(6);

int status = 0;

//practice 6
bool status_1_check = false;
bool status_2_check = false;
bool status_3_check = false;

namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        //* Index setting for each joint
        
        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}

MatrixXd getTransformI0(){
    
    MatrixXd tmp_m(4,4);
    
    //* 1
    tmp_m = MatrixXd::Identity(4,4);
    
    //* 2 
//    tmp_m << 1, 0, 0, 0, \
//             0, 1, 0, 0, \
//             0, 0, 1, 0, \
//             0, 0, 0, 1;
    
    return tmp_m;
}

MatrixXd getTransform6E(){
    MatrixXd tmp_m(4,4);
    tmp_m = MatrixXd::Identity(4,4);
    tmp_m  << 1, 0, 0, 0, \
              0, 1, 0, 0, \
              0, 0, 1, -0.09, \
              0, 0, 0, 1;

    return tmp_m;
}

MatrixXd jointToTransform01(VectorXd q, std::string leg="L"){
    // q: generalize coordinates, q = [q1,q2,q3]
    
    MatrixXd tmp_m(4,4);
    double qq = q(0);
    double c11,c12,c13, c21,c22,c23, c31,c32,c33;
    double rx, ry, rz;
    c11 = cos(qq),  c12 =-sin(qq) , c13 = 0, \
    c21 = sin(qq), c22 = cos(qq), c23 = 0, \
    c31 = 0,        c32 = 0      , c33 = 1 ;
    rx = 0, ry = 0.105, rz = -0.1512;
    if(leg=="R") ry = -ry;
    tmp_m << c11, c12, c13, rx, \
             c21, c22, c23, ry, \
             c31, c32, c33, rz, \
               0,   0,   0,  1;
    return tmp_m;
}

MatrixXd jointToTransform12(VectorXd q){
    // q: generalize coordinates, q = [q1,q2,q3]
    
    MatrixXd tmp_m(4,4);
    double qq = q(1);
    double c11,c12,c13, c21,c22,c23, c31,c32,c33;
    double rx, ry, rz;
    c11 = 1,        c12 =0       , c13 = 0, \
    c21 = 0,        c22 = cos(qq), c23 = -sin(qq), \
    c31 = 0,        c32 = sin(qq)      , c33 = cos(qq) ;
    rx = 0, ry = 0, rz = 0;

    tmp_m << c11, c12, c13, rx, \
             c21, c22, c23, ry, \
             c31, c32, c33, rz, \
               0,   0,   0,  1;
    return tmp_m;
}
MatrixXd jointToTransform23(VectorXd q){
    // q: generalize coordinates, q = [q1,q2,q3]
    
    MatrixXd tmp_m(4,4);
    double qq = q(2);
    double c11,c12,c13, c21,c22,c23, c31,c32,c33;
    double rx, ry, rz;
    c11 = cos(qq),        c12 =0       , c13 = sin(qq),\
    c21 = 0,              c22 = 1,      c23 = 0, \
    c31 = -sin(qq),        c32 = 0      , c33 = cos(qq) ;
    rx = 0, ry = 0, rz = 0;

    tmp_m << c11, c12, c13, rx, \
             c21, c22, c23, ry, \
             c31, c32, c33, rz, \
               0,   0,   0,  1;
    return tmp_m;
}

MatrixXd jointToTransform34(VectorXd q){
    // q: generalize coordinates, q = [q1,q2,q3]
    
    MatrixXd tmp_m(4,4);
    double qq = q(3);
    double c11,c12,c13, c21,c22,c23, c31,c32,c33;
    double rx, ry, rz;
    c11 = cos(qq),        c12 =0       , c13 = sin(qq),\
    c21 = 0,              c22 = 1,      c23 = 0, \
    c31 = -sin(qq),        c32 = 0      , c33 = cos(qq) ;
    rx = 0, ry = 0, rz = -0.35;

    tmp_m << c11, c12, c13, rx, \
             c21, c22, c23, ry, \
             c31, c32, c33, rz, \
               0,   0,   0,  1;
    return tmp_m;
}
MatrixXd jointToTransform45(VectorXd q){
    // q: generalize coordinates, q = [q1,q2,q3]
    
    MatrixXd tmp_m(4,4);
    double qq = q(4);
    double c11,c12,c13, c21,c22,c23, c31,c32,c33; // Rotation Matrix element
    double rx, ry, rz; // position vector
    c11 = cos(qq),        c12 =0       , c13 = sin(qq),\
    c21 = 0,              c22 = 1      , c23 = 0, \
    c31 = -sin(qq),       c32 = 0      , c33 = cos(qq) ;
    
    rx = 0, ry = 0, rz = -0.35;

    tmp_m << c11, c12, c13, rx, \
             c21, c22, c23, ry, \
             c31, c32, c33, rz, \
               0,   0,   0,  1;
    return tmp_m;
}
MatrixXd jointToTransform56(VectorXd q){
    // q: generalize coordinates, q = [q1,q2,q3]
    
    MatrixXd tmp_m(4,4);
    double qq = q(5);
    double c11,c12,c13, c21,c22,c23, c31,c32,c33;
    double rx, ry, rz;
    c11 = 1,        c12 =0       , c13 = 0,\
    c21 = 0,              c22 = cos(qq),      c23 = -sin(qq), \
    c31 = 0,        c32 = sin(qq)      , c33 = cos(qq) ;
    rx = 0, ry = 0, rz = 0;

    tmp_m << c11, c12, c13, rx, \
             c21, c22, c23, ry, \
             c31, c32, c33, rz, \
               0,   0,   0,  1;
    return tmp_m;
}



VectorXd jointToPosition(VectorXd q, std::string leg = "L")
{
    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd T_IE(4,4);
    
    if(leg =="L")
    {
        T_IE = getTransformI0()*
            jointToTransform01(q)*
            jointToTransform12(q)*
            jointToTransform23(q)*
            jointToTransform34(q)*
            jointToTransform45(q)*
            jointToTransform56(q)*
            getTransform6E();
    }
    else if(leg =="R")
    {
        T_IE = getTransformI0()*
            jointToTransform01(q,"R")*
            jointToTransform12(q)*
            jointToTransform23(q)*
            jointToTransform34(q)*
            jointToTransform45(q)*
            jointToTransform56(q)*
            getTransform6E();
    }



    // Block operation manual:   http://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
    tmp_v = T_IE.block(0,3,3,1); // starting at (0,3), block size (3,1)
    
    return tmp_v;
}



MatrixXd jointToRotMat(VectorXd q, std::string leg = "L")
{
    MatrixXd tmp_m = VectorXd::Zero(3);
    MatrixXd T_IE(4,4);
    
    if(leg == "L")
    {
        T_IE = getTransformI0()*
                jointToTransform01(q)*
                jointToTransform12(q)*
                jointToTransform23(q)*
                jointToTransform34(q)*
                jointToTransform45(q)*
                jointToTransform56(q)*
                getTransform6E();
    }
    else if(leg=="R")
    {
        T_IE = getTransformI0()*
                jointToTransform01(q,"R")*
                jointToTransform12(q)*
                jointToTransform23(q)*
                jointToTransform34(q)*
                jointToTransform45(q)*
                jointToTransform56(q)*
                getTransform6E();
    }
    
    tmp_m = T_IE.block(0, 0, 3, 3);
    
    return tmp_m;
}



VectorXd rotToEuler(MatrixXd rotMat)
{
    // EulerZYX
    
    VectorXd eulerZYX(3);
    double z,y,x;
    z = atan2(rotMat(1,0), rotMat(0,0));
    y = atan2( -rotMat(2,0), sqrt(pow(rotMat(2,1), 2.0) + pow(rotMat(2,2), 2.0)) );
    x = atan2(rotMat(2,1), rotMat(2,2));
    eulerZYX << z, y, x; 
//    std::cout << eulerZYX << endl;
    
    return eulerZYX;
}
MatrixXd jointToPosJac(VectorXd q, std::string leg = "L")
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    Vector3d r_I_IE;


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    if(leg != "R" && leg != "L")
    {
        std::cout << "leg value invalid error\n";
    }
    T_01 = jointToTransform01(q,leg);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;
    T_I3 = T_I0*T_01*T_12*T_23;
    T_I4 = T_I0*T_01*T_12*T_23*T_34;
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;
    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.
    r_I_IE = (T_I6*T_6E).block(0,3,3,1);


    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);

//    std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd jointToRotJac(VectorXd q, std::string leg = "L")
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    if(leg != "R" && leg != "L")
    {
        std::cout << "leg value invalid error\n";
    }
    T_01 = jointToTransform01(q,leg);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;
    T_I3 = T_I0*T_01*T_12*T_23;
    T_I4 = T_I0*T_01*T_12*T_23*T_34;
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;
    
    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the translational Jacobian.
    J_R.col(0) << R_I1*n_1;
    J_R.col(1) << R_I2*n_2;
    J_R.col(2) << R_I3*n_3;
    J_R.col(3) << R_I4*n_4;
    J_R.col(4) << R_I5*n_5;
    J_R.col(5) << R_I6*n_6;

//    std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd pseudoInverseMat(MatrixXd A, double lambda)
{
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA, tmp;
    
    int m = A.rows();
    int n = A.cols();
    if(m >= n)
    {
        tmp = A.transpose() * A + lambda * lambda * MatrixXd::Identity(n,n);
        pinvA = tmp.inverse() * A.transpose();
    }
    else
    {
        tmp = A*A.transpose() + lambda * lambda * MatrixXd::Identity(m, m);
        pinvA = A.transpose() * tmp.inverse();
    }
//    std::cout << "pinvA : " << pinvA << std::endl;


    return pinvA;
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C
    Vector3d phi,n;
    double th;
    
    th = acos((C(0,0) + C(1,1) + C(2,2) -1) / 2);
    
    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{
        n<< (C(2,1) - C(1,2)),
            (C(0,2) - C(2,0)),
            (C(1,0) - C(0,1));
        n = n * 1/(2*sin(th));
    }
        
    phi = th*n;
    
   
    return phi;
}

VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol, std::string leg = "L")
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    
    double num_it;
    MatrixXd J_P(6,6), J_R(6,6), J(6,6), pinvJ(6,6), C_err(3,3), C_IE(3,3);
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    // leg value valid check 
    if(leg != "R" && leg != "L")
    {
        std::cout << "leg value invalid error\n";
    }
    //* Initialize the solution with the initial guess
    q = q0;
    C_IE = jointToRotMat(q,leg);
    C_err = C_des * C_IE.transpose();// orientation error
    
    //* Damping factor
    lambda = 0.001;
    
    //* Initialize error
    dr = r_des - jointToPosition(q,leg);
    dph = rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        
        //Compute Inverse Jacobian
        J_P = jointToPosJac(q,leg);
        J_R = jointToRotJac(q,leg);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        C_IE = jointToRotMat(q,leg);
        C_err = C_des * C_IE.transpose();
        
        dr = r_des - jointToPosition(q,leg);
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
                   
        num_it++;
    }
//    std::cout << "iteration: " << num_it << ", value: " << q*R2D << std::endl;
    
    return q;
}
double func_1_cos(double t, double init, double final, double T){
    double des;
    des = (final - init) * 0.5 * (1.0 - cos(PI*(t/T))) + init;
    return des;
}

void Practice()
{
    MatrixXd J_R(3,6),J_P = MatrixXd::Zero(3,6);
    MatrixXd TI0(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4),T45(4,4),T56(4,4),T6E(4,4),TIE(4,4), CIE(3,3);
    VectorXd q(6),q_cal(6); // Vector3D can't create 6D array
    Vector3d r_des;
    MatrixXd C_des;
//    q << 10, 20, 30, 40, 50, 60;
//    for(int i = 0; i<6; i++)
//    {
//        q[i] *= D2R;
//    }
    
    q << 0, 0, -30, 60, -30, 0;
    for(int i = 0; i<6; i++)
    {
        q[i] *= D2R;
    }
    Vector3d pos, euler;

    r_des << 0,
             0.105,
            -0.55;
    C_des = MatrixXd::Identity(3,3);
//    C_des << 1,0,0,\
//             0,1,0,\
//             0,0,1;

    
    q_cal = inverseKinematics(r_des, C_des, q*0.5, 0.001);
    q_global = q_cal;
    
}
// right leg IK, FK create

void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("/home/culigan3186/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();


    //* setting for getting dt
    last_update_time = model->GetWorld()->GetSimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));
    
    // Practice();
}


void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */
    
    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->GetSimTime();
    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
    GetjointData();
    
    //* Target Angles
//    joint[LHY].targetRadian = q_global(0); // 
//    joint[LHR].targetRadian = q_global(1); // 
//    joint[LHP].targetRadian = q_global(2); //
//    joint[LKN].targetRadian = q_global(3); // 
//    joint[LAP].targetRadian = q_global(4); 
//    joint[LAR].targetRadian = q_global(5);
    //* Joint Controller
    
    
    if(status==0)
    {
        status = 1;
        r_des << 0, 0.105, -0.55;
        C_des = MatrixXd::Identity(3,3);
        q << 0, 0, -30*D2R, 60*D2R, -30*D2R, 0; 
        tar_q_left = inverseKinematics(r_des, C_des, q, 0.001);
    }
    

    if(time < 5) // status = 0 
    {   
        q_left_cmd(0) = func_1_cos(time,0,tar_q_left(0),5);
        q_left_cmd(1) = func_1_cos(time,0,tar_q_left(1),5);
        q_left_cmd(2) = func_1_cos(time,0,tar_q_left(2),5);
        q_left_cmd(3) = func_1_cos(time,0,tar_q_left(3),5);
        q_left_cmd(4) = func_1_cos(time,0,tar_q_left(4),5);
        q_left_cmd(5) = func_1_cos(time,0,tar_q_left(5),5);   
    }
    
    else if(status >=1 && !status_1_check && time >= 5)
    {
        q = tar_q_left;
        r_des << 0, 0.105, -0.35;
        tar_q_left = inverseKinematics(r_des, C_des, q, 0.001);
        status_1_check = true;
        status = 2;
        // status = 3;
    }
    else if(status >= 1  && time <= 10)
    {

        double time1 = time - 5;
    
        q_left_cmd(0) = func_1_cos(time1, q(0), tar_q_left(0), 5);
        q_left_cmd(1) = func_1_cos(time1, q(1), tar_q_left(1), 5);
        q_left_cmd(2) = func_1_cos(time1, q(2), tar_q_left(2), 5);
        q_left_cmd(3) = func_1_cos(time1, q(3), tar_q_left(3), 5);
        q_left_cmd(4) = func_1_cos(time1, q(4), tar_q_left(4), 5);
        q_left_cmd(5) = func_1_cos(time1, q(5), tar_q_left(5), 5);
    }
    
    else if (status == 2 && time <= 15 && !status_2_check)
    {
        q = tar_q_left;
        r_des << 0, 0.105, -0.55;
        tar_q_left = inverseKinematics(r_des, C_des, q, 0.001);
        status_2_check = true;
    }
    else if (status == 3 && time <=15 && !status_3_check)
    {
        MatrixXd tmp;
        double rot;
        rot = 90*D2R;
        q = tar_q_left;
        r_des << 0, 0.105, -0.35;
        tmp   <<   cos(rot), -sin(rot), 0,
                   sin(rot),  cos(rot), 0,
                             0,            0, 1;
        C_des = tmp;
        tar_q_left = inverseKinematics(r_des, C_des, q, 0.001);
        status_3_check = true;
    }
    else if(status == 2 && time <= 15)
    {   
        double time2 = time - 10.0;
        
        q_left_cmd(0) = func_1_cos(time2,q(0),tar_q_left(0),5);
        q_left_cmd(1) = func_1_cos(time2,q(1),tar_q_left(1),5);
        q_left_cmd(2) = func_1_cos(time2,q(2),tar_q_left(2),5);
        q_left_cmd(3) = func_1_cos(time2,q(3),tar_q_left(3),5);
        q_left_cmd(4) = func_1_cos(time2,q(4),tar_q_left(4),5);
        q_left_cmd(5) = func_1_cos(time2,q(5),tar_q_left(5),5);
        
    }
    
    else if(status == 3 && time <= 15)
    {
        double time3 = time - 10.0;   
        q_left_cmd(0) = func_1_cos(time3,q(0),tar_q_left(0),5);
        q_left_cmd(1) = func_1_cos(time3,q(1),tar_q_left(1),5);
        q_left_cmd(2) = func_1_cos(time3,q(2),tar_q_left(2),5);
        q_left_cmd(3) = func_1_cos(time3,q(3),tar_q_left(3),5);
        q_left_cmd(4) = func_1_cos(time3,q(4),tar_q_left(4),5);
        q_left_cmd(5) = func_1_cos(time3,q(5),tar_q_left(5),5);
   
    }
    
//    std::cout << "status : " << status << "\n";
   for(int i=0; i<6; i++)
   {
       std::cout << "q_left_cmd(" << i << ") = " << q_left_cmd(i) << '\n';
    //    std::cout << "tar_q_left(" << i << "::::: " << tar_q_left(i) << '\n';
   }
    
    joint[LHY].targetRadian = q_left_cmd(0);
    joint[LHR].targetRadian = q_left_cmd(1);
    joint[LHP].targetRadian = q_left_cmd(2);
    joint[LKN].targetRadian = q_left_cmd(3);
    joint[LAP].targetRadian = q_left_cmd(4);
    joint[LAR].targetRadian = q_left_cmd(5);
    
    
    jointController();
}

void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian-joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity-joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);

    torso_joint->SetForce(0, joint[WST].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");

    //* FTsensor joint
    LS = this->model->GetJoint("LS");
    RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->GetAngle(0).Radian();
    joint[LHR].actualRadian = L_Hip_roll_joint->GetAngle(0).Radian();
    joint[LHP].actualRadian = L_Hip_pitch_joint->GetAngle(0).Radian();
    joint[LKN].actualRadian = L_Knee_joint->GetAngle(0).Radian();
    joint[LAP].actualRadian = L_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[LAR].actualRadian = L_Ankle_roll_joint->GetAngle(0).Radian();

    joint[RHY].actualRadian = R_Hip_yaw_joint->GetAngle(0).Radian();
    joint[RHR].actualRadian = R_Hip_roll_joint->GetAngle(0).Radian();
    joint[RHP].actualRadian = R_Hip_pitch_joint->GetAngle(0).Radian();
    joint[RKN].actualRadian = R_Knee_joint->GetAngle(0).Radian();
    joint[RAP].actualRadian = R_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[RAR].actualRadian = R_Ankle_roll_joint->GetAngle(0).Radian();

    joint[WST].actualRadian = torso_joint->GetAngle(0).Radian();

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    joint[LHY].Kp = 2000;
    joint[LHR].Kp = 9000;
    joint[LHP].Kp = 2000;
    joint[LKN].Kp = 5000;
    joint[LAP].Kp = 3000;
    joint[LAR].Kp = 3000;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[WST].Kp = 2.;

    joint[LHY].Kd = 2.;
    joint[LHR].Kd = 2.;
    joint[LHP].Kd = 2.;
    joint[LKN].Kd = 4.;
    joint[LAP].Kd = 2.;
    joint[LAR].Kd = 2.;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;

    joint[WST].Kd = 2.;
}

