#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "rosgraph_msgs/Clock.h"
#include <string>
#include <iostream>
#include "boost/thread.hpp"
#include <chrono>

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <ros/package.h>

#include "std_msgs/Float64MultiArray.h"

//Include Tf libraries
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

//Include Simulink-generated library
#include "CLIK.h"   

//Include other libraries
#include "modelBasedControlPlanarDualArmDroneV3.hpp"
#include "utils_2.hpp"
#include "dynamicModelPlanarDualArmDroneV3.hpp"
#include "trajectoryGeneration.hpp"


#define RATE 100
#define def_load 0
#define def_K_second 1000
#define def_q1l_n 0
#define def_q2l_n 0
#define def_q3l_n 0
#define def_q4l_n -1.5
#define def_q1r_n 0
#define def_q2r_n 0
#define def_q3r_n 0
#define def_q4r_n -1.5
#define NJ 4

using namespace std;

class CLIK_NODE {
    public:
        CLIK_NODE();

        void joint_states_cb( sensor_msgs::JointState );
        void move_arms_cb( std_msgs::Float64 );
        void grip_cb( std_msgs::Float64 ); 
        void x_drone_state_cb( std_msgs::Float64MultiArray ); 

        void goto_initial_position( double dp_l[NJ], double dp_r[NJ] );
        void ctrl_loop();
        void run();

        void Bfun(Eigen::MatrixXd& B, Eigen::VectorXd q);
        void nfun(Eigen::VectorXd& n, Eigen::VectorXd q, Eigen::VectorXd qDot);

    private:
        ros::NodeHandle _nh;

        //Simulink-generated object for inverse kinematics
        CLIK rtObj;

        //state variables
        double xd[3];
        double ql[NJ];
        double qr[NJ];

        double CoM[3];
        double CoM_bar[3];

        double ql_d[NJ];
        double qr_d[NJ];

        bool _move_arms;

        double L_half;

        //Variables for oscillation reduction
        double _jDronePos;
        double _jCablesPos;         // Total angle
        double _jCablesPos1;        // cables angle
        double _jCablesPos2;        // drone angle
        double _jLeftArmPos;
        double _jRightArmPos;
        double _jShouldersPos;
        double _jDroneVel;
        double _jCablesVel;        
        double _jCablesVel1;
        double _jCablesVel2;
        double _jLeftArmVel;
        double _jRightArmVel;
        double _jShouldersVel;

        //ROS topic objects
        ros::Subscriber _js_sub;
        ros::Subscriber _move_arms_sub;
        ros::Subscriber _grip_sub;
        ros::Subscriber _x_drone_state_sub;
		ros::Publisher _left_cmd_pub[NJ];
        ros::Publisher _right_cmd_pub[NJ];
        ros::Publisher _activate_joywrap;

        ros::Publisher _estimate_pub;
        ros::Publisher _error_pub;

        //TF objects
        tf::TransformListener _listener;
        tf::StampedTransform _tf_ref;

		tf::TransformBroadcaster _trans_br;

};

CLIK_NODE::CLIK_NODE() {
    _js_sub = _nh.subscribe("/licasa1/joint_states", 0, &CLIK_NODE::joint_states_cb, this);
    _move_arms_sub = _nh.subscribe("/licasa1/move_arms", 0, &CLIK_NODE::move_arms_cb, this);
    _grip_sub = _nh.subscribe("/licasa1/L_half", 0, &CLIK_NODE::grip_cb, this);
    _x_drone_state_sub = _nh.subscribe("/licasa1/x_drone_state", 0, &CLIK_NODE::x_drone_state_cb, this);
    _error_pub = _nh.advertise< std_msgs::Float64 > ("/licasa1/error", 1);

    // Added to initialize /licasa1/estimate for rqt
    _estimate_pub = _nh.advertise< std_msgs::Float64MultiArray > ("/licasa1/estimate", 1);
    std_msgs::Float64MultiArray est_msg;
    est_msg.data.clear();
    for (int i = 0; i < 6; i++) {
        est_msg.data.push_back(0);
    }
    _estimate_pub.publish(est_msg); 

    _move_arms = false;

    _left_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_1_effort_pos_controller/command", 1);
	_left_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_2_effort_pos_controller/command", 1);
	_left_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_3_effort_pos_controller/command", 1);
	_left_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_4_effort_pos_controller/command", 1);

    _right_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_1_effort_pos_controller/command", 1);
	_right_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_2_effort_pos_controller/command", 1);
	_right_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_3_effort_pos_controller/command", 1);
	_right_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_4_effort_pos_controller/command", 1);

    _activate_joywrap = _nh.advertise< std_msgs::Float64 > ("/licasa1/joywrap_start", 1);

    ql[0] = def_q1l_n;
    ql[1] = def_q2l_n;
    ql[2] = def_q3l_n;
    ql[3] = def_q4l_n;
    qr[0] = def_q1r_n;
    qr[1] = def_q2r_n;
    qr[2] = def_q3r_n;
    qr[3] = def_q4r_n;

    ql_d[0] = def_q1l_n;
    ql_d[1] = def_q2l_n;
    ql_d[2] = def_q3l_n;
    ql_d[3] = def_q4l_n;
    qr_d[0] = def_q1r_n;
    qr_d[1] = def_q2r_n;
    qr_d[2] = def_q3r_n;
    qr_d[3] = def_q4r_n;

    xd[0] = 0.2494;
    xd[1] = 0;
    xd[2] = -0.2927;
    L_half = 0.18;

    rtObj.initialize();

    rtObj.rtU.load = def_load;
    rtObj.rtU.K_second = def_K_second;
    rtObj.rtU.L_half = L_half;

    rtObj.rtU.q1l_n = def_q1l_n;
    rtObj.rtU.q2l_n = def_q2l_n;
    rtObj.rtU.q3l_n = def_q3l_n;
    rtObj.rtU.q4l_n = def_q4l_n;
    rtObj.rtU.q1r_n = def_q1r_n;
    rtObj.rtU.q2r_n = def_q2r_n;
    rtObj.rtU.q3r_n = def_q3r_n;
    rtObj.rtU.q4r_n = def_q4r_n;

    _jDronePos = 0;
    _jCablesPos = 0;
    _jCablesPos1 = 0;
    _jCablesPos2 = 0;
    _jLeftArmPos = 0;
    _jRightArmPos = 0;
    _jShouldersPos = 0;
    _jDroneVel = 0;
    _jCablesVel = 0;
    _jCablesVel2 = 0;
    _jLeftArmVel = 0;
    _jRightArmVel = 0;
    _jShouldersVel = 0;
}

void CLIK_NODE::Bfun(Eigen::MatrixXd& B, Eigen::VectorXd q) {
  B_row1(B,q);
  B_row2(B,q);
  B_row3(B,q);
  B_row4(B,q);
}

void CLIK_NODE::nfun(Eigen::VectorXd& n, Eigen::VectorXd q, Eigen::VectorXd qDot) {
  n_row1(n,q,qDot);
  n_row2(n,q,qDot);
  n_row3(n,q,qDot);
  n_row4(n,q,qDot);
}

//Callback for the joint state
void CLIK_NODE::joint_states_cb( sensor_msgs::JointState js ) {

    if (js.name[0]!="licasa1/rotor_0_joint") {
        //We assume to know the number of joints
        for(int i=0; i<NJ; i++ ) {
            ql[i] = js.position[i];
            qr[i] = js.position[i+NJ];
            //cout<<"I HEARD "<<ql[i]<<" AND "<<qr[i]<<endl;
        }

        _jCablesPos1 = js.position[9];
        _jLeftArmPos = js.position[0];
        _jRightArmPos = js.position[4];
        _jShouldersPos = js.position[12];
        _jCablesVel1 = js.velocity[9];
        _jLeftArmVel = js.velocity[0];
        _jRightArmVel = js.velocity[4];
        _jShouldersVel = js.velocity[12];

    } else {
        //cout<<js.name[0]<<endl;
    }

}

void CLIK_NODE::move_arms_cb( std_msgs::Float64 f) {
    if (f.data > 0) {
        _move_arms = true;
        // cout<<"ARMS MOVING\n";
    } else {
        _move_arms = false;
        // cout<<"ARMS NOT MOVING\n";
    }
}

void CLIK_NODE::x_drone_state_cb( std_msgs::Float64MultiArray f) {
    _jDronePos = f.data[0];
    _jDroneVel = f.data[1];
    _jCablesPos2 = f.data[2];
    _jCablesVel2 = f.data[3];
}

void CLIK_NODE::grip_cb( std_msgs::Float64 f) {
    L_half = f.data;
}

void CLIK_NODE::goto_initial_position( double dp_l[NJ], double dp_r[NJ] ) {
    ros::Rate r(100);

	float min_e = 1000.0;
	float left_max_e = 1000.0;
    float right_max_e = 1000.0;

	std_msgs::Float64 left_cmd[NJ];
    std_msgs::Float64 right_cmd[NJ];

	//While the maximum error over all the left joints is higher than a given threshold 
	while( left_max_e > 0.05 || right_max_e > 0.05) {
 		left_max_e = -1000;
        right_max_e = -1000;
		//Command the same value for all the joints and calculate the maximum error
		for(int i=0; i<NJ; i++) {
 			left_cmd[i].data = dp_l[i];
			_left_cmd_pub[i].publish (left_cmd[i]);
			float left_e = fabs( left_cmd[i].data - ql[i] );
			//max_e is the maximum error over all the joints
			left_max_e = ( left_e > left_max_e ) ? left_e : left_max_e;
            cout<<"Left error: "<<left_max_e<<endl;
        }
        for(int i=0; i<NJ; i++) {
            right_cmd[i].data = dp_r[i];
			_right_cmd_pub[i].publish (right_cmd[i]);
			float right_e = fabs( right_cmd[i].data - qr[i] );
			//max_e is the maximum error over all the joints
			right_max_e = ( right_e > right_max_e ) ? right_e : right_max_e;
            cout<<"Right error: "<<right_max_e<<endl;
		}
		r.sleep();
	}

	sleep(2);
    
    cout<<"CLIK STARTED JOYWRAP\n";

}

void CLIK_NODE::ctrl_loop() {
    ros::Rate r(RATE);

    //Control the robot towards a fixed initial position
	double left_i_cmd[NJ];
	left_i_cmd[0] = 0;
	left_i_cmd[1] = 0;
	left_i_cmd[2] = 0;
	left_i_cmd[3] = 0;

    double right_i_cmd[NJ];
	right_i_cmd[0] = 0;
	right_i_cmd[1] = 0;
	right_i_cmd[2] = 0;
	right_i_cmd[3] = 0;

    //Lock the code to start manually the execution of the trajectory
	cout << "Press enter to start the trajectory execution" << endl;
	string ln;
	getline(cin, ln);

    goto_initial_position(left_i_cmd, right_i_cmd);
    cout<<"Initial position reached\n";

    left_i_cmd[0] = def_q1l_n;
	left_i_cmd[1] = def_q2l_n;
	left_i_cmd[2] = def_q3l_n;
	left_i_cmd[3] = def_q4l_n;
    right_i_cmd[0] = def_q1r_n;
	right_i_cmd[1] = def_q2r_n;
	right_i_cmd[2] = def_q3r_n;
	right_i_cmd[3] = def_q4r_n;

    // Activate the JoyWrap
    std_msgs::Float64 joywrap_msg;
    joywrap_msg.data = 1;

    while (!_listener.waitForTransform("shoulder_link_y", "ref_frame", ros::Time(0), ros::Duration(0.01))){
        // cout<<"Waiting for reference\n";
        _activate_joywrap.publish(joywrap_msg);
        r.sleep();
	}

    bool first_time = true;
    int oo = 0;
    std::vector<double> qCSaved, qLSaved, qRSaved, tau1Saved, tau2Saved, uSaved, qLAccSaved, qRAccSaved, qLVelSaved, qRVelSaved, eSaved, qCVelSaved;
    std::vector<double> tDes, qDesPassive, qDotDesPassive, qDDotDesPassive;
    double err = 0;
    double timeStep = 0.01;
    double timeFinal = 6;
    Eigen::VectorXd qPassive(1);
    Eigen::VectorXd qDotPassive(1);
    Eigen::VectorXd qActive(2);
    Eigen::VectorXd qDotActive(2);
    Eigen::VectorXd in(1);
    Eigen::VectorXd y(2);
    Eigen::MatrixXd Kp(1,1);
    Eigen::MatrixXd Kd(1,1);
    Eigen::MatrixXd Gp(2,2);
    Eigen::MatrixXd Gd(2,2);
    Eigen::MatrixXd B(4,4);
    Eigen::VectorXd n(4);

    Eigen::MatrixXd Bb(1,1);
    Eigen::MatrixXd Bbm(1,2);
    Eigen::MatrixXd pinvBbm(2,1);
    Eigen::MatrixXd Bmb(2,1);
    Eigen::MatrixXd Bm(2,2);
    Eigen::VectorXd nb(1);
    Eigen::VectorXd nm(2);
    Eigen::VectorXd B_hat(2);
    Eigen::VectorXd n_hat(2);
    Eigen::MatrixXd B_tilde(2,2);
    Eigen::VectorXd n_tilde(2);
    Eigen::VectorXd tau_tilde(4);
    
    Eigen::VectorXd e(1);
    Eigen::VectorXd eDot(1);
    Eigen::VectorXd tau(2);

    std_msgs::Float64 jLeftCommand;
    std_msgs::Float64 jRightCommand;
    std_msgs::Float64 PosCommand;

    Eigen::VectorXd acc(4);
    Eigen::VectorXd vel(4);
    Eigen::VectorXd pos(4);
    acc = Eigen::VectorXd::Zero(4);
    vel = Eigen::VectorXd::Zero(4);
    pos = Eigen::VectorXd::Zero(4);

    Kp.diagonal() << 1;
    Kd.diagonal() << 0.01;
    Gp.diagonal() << 1, 1;
    Gd.diagonal() << 0.01, 0.01;

    cout << "Press enter to start the trajectory execution" << endl;
	getline(cin, ln);

    while(ros::ok()) {
        //_activate_joywrap.publish(joywrap_msg);
        _listener.lookupTransform("shoulder_link_y","ref_frame",ros::Time(0), _tf_ref);

        xd[0] = _tf_ref.getOrigin().x();
        xd[1] = _tf_ref.getOrigin().y();
        xd[2] = _tf_ref.getOrigin().z();

        rtObj.rtU.xd[0] = xd[0];
        rtObj.rtU.xd[1] = xd[1];
        rtObj.rtU.xd[2] = xd[2];

        rtObj.rtU.q1l = ql[0];
        rtObj.rtU.q2l = ql[1];
        rtObj.rtU.q3l = ql[2];
        rtObj.rtU.q4l = ql[3];
        rtObj.rtU.q1r = qr[0];
        rtObj.rtU.q2r = qr[1];
        rtObj.rtU.q3r = qr[2];
        rtObj.rtU.q4r = qr[3];
        rtObj.rtU.L_half = L_half;

        //cout<<"Move arms: "<<_move_arms<<endl;
        
        // If _move_arms is false CLIK isn't called, so the joints keep on following the previous reference
        if (_move_arms == true) { 
            first_time = true;
            rtObj.step();

            std_msgs::Float64 left_cmd[NJ];
            std_msgs::Float64 right_cmd[NJ];

            left_cmd[0].data = rtObj.rtY.q1l_d;
            left_cmd[1].data = rtObj.rtY.q2l_d;
            left_cmd[2].data = rtObj.rtY.q3l_d;
            left_cmd[3].data = rtObj.rtY.q4l_d;
            right_cmd[0].data = rtObj.rtY.q1r_d;
            right_cmd[1].data = rtObj.rtY.q2r_d;
            right_cmd[2].data = rtObj.rtY.q3r_d;
            right_cmd[3].data = rtObj.rtY.q4r_d;

            //cout<<"LEFT CMD: \n"<<rtObj.rtY.q1l_d<<endl<<rtObj.rtY.q2l_d<<endl<<rtObj.rtY.q3l_d<<endl<<rtObj.rtY.q4l_d<<endl;
            //cout<<"RIGHT CMD: \n"<<rtObj.rtY.q1r_d<<endl<<rtObj.rtY.q2r_d<<endl<<rtObj.rtY.q3r_d<<endl<<rtObj.rtY.q4r_d<<endl;

            // cout<<"Error: "<<rtObj.rtY.err<<endl;

            //Publish all the commands in topics
            for(int i=0; i<NJ; i++) {
                _left_cmd_pub[i].publish (left_cmd[i]);
                _right_cmd_pub[i].publish (right_cmd[i]);
            }
        } else {
            // Oscillation reduction code
            // cout<<"Starting oscillation reduction\n";

            // ros::Duration(2).sleep();
            if (first_time) {

                cubicVelTraj(tDes, qDesPassive, qDotDesPassive, qDDotDesPassive, timeStep, timeFinal, 0.0, 0.0, 0.0, 0.0);

                oo = 0;
                first_time = false;
                cout<<"First time oscillations\n";
            }
            if (oo < qDesPassive.size()) {

                // cout<<"Not first time oscillations\n";

                Eigen::VectorXd q(5);
                Eigen::VectorXd qDot(5);
                Eigen::VectorXd qInterest(3);
                Eigen::VectorXd qDotInterest(3);
                Eigen::VectorXd qPassive(1);
                Eigen::VectorXd qDotPassive(1);
                Eigen::VectorXd qActive(2);
                Eigen::VectorXd qDotActive(2);

                _jCablesPos = _jCablesPos1 - _jCablesPos2;
                _jCablesVel = _jCablesVel1 - _jCablesVel2;

                // cout<<"Cables angle: "<<_jCablesPos1<<"; Drone angle: "<<_jCablesPos2<<endl;

                // cout<<"jDronePos: "<<_jDronePos<<endl<<"jCablesPos: "<<_jCablesPos<<endl<<"jShouldersPos: "<<_jShouldersPos<<endl<<"jLeftArmPos: "<<_jLeftArmPos<<endl<<"jRightArmPos: "<<_jRightArmPos<<endl;

                q << _jDronePos, -_jCablesPos, _jCablesPos, _jLeftArmPos, _jRightArmPos;
                qDot << _jDroneVel, -_jCablesVel, _jCablesVel, _jLeftArmVel, _jRightArmVel;
                //cout << "q: " << q.transpose() << endl;

                qInterest << q(0), q(1), q(3);
                qDotInterest << qDot(0), qDot(1), qDot(3);

                cout<<"qInterest: "<<qInterest.transpose()<<endl;
                cout<<"qDotInterest: "<<qDotInterest.transpose()<<endl;

                // qInterest << -0.010246, -0.0476165, -1.05927;
                // qDotInterest << -0.0643891, -0.204025, -2.38112;

                qPassive << q(1);
                qDotPassive << qDot(1);

                // cout << "_jCablesPos: " << qPassive.transpose() << endl;

                qActive = q.tail(2);
                qDotActive = qDot.tail(2);

                Eigen::VectorXd qDesPassiveEigen(1);
                Eigen::VectorXd qDotDesPassiveEigen(1);
                Eigen::VectorXd qDDotDesPassiveEigen(1);

                qDesPassiveEigen << qDesPassive[oo];
                qDotDesPassiveEigen << qDotDesPassive[oo];
                qDDotDesPassiveEigen << qDDotDesPassive[oo];

                // Computation auxiliar input
                e = qDesPassiveEigen - qPassive;
                e(0) = e(0) + 0.00194396;             //Correction because at steady state e doesn't go to 0
                // err = err*0.999 + e[0]*0.001;
                // eInt += e * timeStep;


                std::cout << "Error: " << e.transpose() << std::endl;
                std_msgs::Float64 err_msg;
                err_msg.data = e(0);
                _error_pub.publish(err_msg);
                
                // std::cout << "Time: " << oo*timeStep <<std::endl;
                // PosCommand.data = q(0);
                // _PositionPub.publish(PosCommand);
                eDot = qDotDesPassiveEigen - qDotPassive;
                
                in = Kp*e + Kd*eDot + qDDotDesPassiveEigen;
                // cout<<"eDot: "<<eDot.transpose()<<endl;
                // cout<<"qDDotDesPassiveEigen: "<<qDDotDesPassiveEigen.transpose()<<endl;
                // cout<<"In: "<<in.transpose()<<endl;
                
                //if(e[0]<0.01)  return;

                // Computation dynamic model
                B = Eigen::MatrixXd::Zero(4,4);
                n = Eigen::VectorXd::Zero(4);
                Bfun(B,qInterest);
                nfun(n,qInterest,qDotInterest);

                // cout<<"qInterest: "<<qInterest.transpose()<<endl;
                // cout<<"qDotInterest: "<<qDotInterest.transpose()<<endl;
                // cout<<"B: "<<B<<endl;
                // cout<<"n: "<<n<<endl;

                // Matrix decomposition
                Bb = B.block<1,1>(1,1);
                Bbm = B.block<1,2>(1,2);
                Bmb = Bbm.transpose();
                Bm = B.block<2,2>(2,2);
                nb << n(1);
                nm = n.tail(2);
                pinvBbm = pseudoinverse(Bbm,0);

                // Composed matrix computation
                B_hat = Bmb-Bm*pinvBbm*Bb;
                n_hat = nm-Bm*pinvBbm*nb;

                // Composed matrix computation
                B_tilde = Bm - Bmb*Bb.inverse()*Bbm;
                n_tilde = nm - Bmb*Bb.inverse()*nb;

                // cout<<"n_tilde: "<<n_tilde.transpose()<<endl;

                // Control law - Damped pseudoinverse
                y = -pinvBbm*(Bb*in+nb) + (Eigen::MatrixXd::Identity(2,2)-pinvBbm*Bbm)*(Gd*(-qDotActive) + Gp*(-qActive));
                tau = B_tilde*y + n_tilde;
                //tau = B_hat*in + n_hat;
            
                // cout << "tau " << tau.transpose() << endl;
                //Torque -> position

                tau_tilde << 0, 0, tau;                                                        //Inverto il modello dinamico
                acc=B.inverse()*(tau_tilde-n);

                vel += acc*timeStep;                                                        //Metodo di integrazione classico

                if(vel[2]<=-1.7)vel[2]=-1.7;                                                //Saturazione velocità
                if(vel[2]>=1.7) vel[2]=1.7;
                if(vel[3]>=1.7) vel[3]=1.7;
                if(vel[3]<=-1.7)vel[3]=-1.7;

                pos += vel*timeStep;    

                //Output saturation:
                if(pos[2]<=-1.57079632679) pos[2]=-1.57079632679;        //verso dentro fino a 10°
                if(pos[2]>=1.57079632679) pos[2]=1.57079632679;          //verso fuori fino a 90°
                if(pos[3]>=1.57079632679) pos[3]=1.57079632679;
                if(pos[3]<=-1.57079632679) pos[3]=-1.57079632679;
                
                // std::cout << "acc: " << acc.transpose() << std::endl;
                // std::cout << "vel: " << vel.transpose() << std::endl;
                // std::cout << "pos: " << pos.transpose() << std::endl;

                //Printing output
                Eigen::VectorXd q0(2);
                Eigen::VectorXd q0d(2);
                q0 << pos[2], pos[3];
                q0d << vel[2], vel[3];
                //cout << "pos: " << q0.transpose() << endl;
                //cout << "vel: " << q0d.transpose() << endl;

                // Publishing the commands
                jLeftCommand.data = pos[2];            //rad    (left +, right -)
                jRightCommand.data = pos[3];

                // jLeftCommand.data = in[0];            
                // jRightCommand.data = in[0];

                //jLeftCommand.data = 0.3;
                //jRightCommand.data = -0.3;
                _left_cmd_pub[0].publish(jLeftCommand);
                _right_cmd_pub[0].publish(jRightCommand);
                // cout<<pos[2]<<"; "<<pos[3]<<endl;
                
                oo++;
                cout<<endl;
                // if (oo == 5) {
                //     return;
                // }
                
                // cout<<"Increased oo counter\n";
                if (oo == qDesPassive.size() - 1) {
                    oo--;
                    // cout<<"Decreased oo counter\n";
                }
            }
        }

        for(int i = 0; i<3; i++) {
            CoM[i] = rtObj.rtY.CoM[i];
            CoM_bar[i] = rtObj.rtY.CoM_bar[i];
        }

        tf::Transform CoM_trans;
        tf::Transform CoM_bar_trans;

        CoM_trans.setOrigin(tf::Vector3(CoM[0], CoM[1], CoM[2]));
        CoM_bar_trans.setOrigin(tf::Vector3(CoM_bar[0], CoM_bar[1], CoM_bar[2]));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        CoM_trans.setRotation(q);
        CoM_bar_trans.setRotation(q);

        _trans_br.sendTransform(tf::StampedTransform(CoM_trans, ros::Time::now(), "shoulder_link_y", "CoM"));
		_trans_br.sendTransform(tf::StampedTransform(CoM_bar_trans, ros::Time::now(), "shoulder_link_y", "CoM_bar"));

		r.sleep();
	}

}

void CLIK_NODE::run() {
    boost::thread( &CLIK_NODE::ctrl_loop, this);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "clik_ctrl");
	CLIK_NODE cn;
	cout<<"Constructor worked\n";

    cn.run();

    ros::spin();

	return 0;
}


