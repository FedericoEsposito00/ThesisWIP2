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
#include <std_msgs/Float64.h>
#include <ros/package.h>

#include "std_msgs/Float64MultiArray.h"

//Include Tf libraries
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

//Include Simulink-generated library
#include "CLIK.h"   

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
        void goto_initial_position( double dp_l[NJ], double dp_r[NJ] );
        void ctrl_loop();
        void run();

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

        //ROS topic objects
        ros::Subscriber _js_sub;
        ros::Subscriber _move_arms_sub;
        ros::Subscriber _grip_sub;
		ros::Publisher _left_cmd_pub[NJ];
        ros::Publisher _right_cmd_pub[NJ];
        ros::Publisher _activate_joywrap;

        ros::Publisher _estimate_pub;

        //TF objects
        tf::TransformListener _listener;
        tf::StampedTransform _tf_ref;

		tf::TransformBroadcaster _trans_br;

};

CLIK_NODE::CLIK_NODE() {
    _js_sub = _nh.subscribe("/licasa1/joint_states", 0, &CLIK_NODE::joint_states_cb, this);
    _move_arms_sub = _nh.subscribe("/licasa1/move_arms", 0, &CLIK_NODE::move_arms_cb, this);
    _grip_sub = _nh.subscribe("/licasa1/L_half", 0, &CLIK_NODE::grip_cb, this);

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
    } else {
        //cout<<js.name[0]<<endl;
    }

}

void CLIK_NODE::move_arms_cb( std_msgs::Float64 f) {
    if (f.data > 0) {
        _move_arms = true;
        cout<<"ARMS MOVING\n";
    } else {
        _move_arms = false;
        cout<<"ARMS NOT MOVING\n";
    }
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
        
        // If _move_arms is false CLIK isn't called, so the joints keep on following the previous reference
        if (_move_arms == true) { 
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


