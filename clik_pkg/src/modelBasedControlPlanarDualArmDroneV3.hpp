#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include <thread>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <stdio.h>
#include <string.h>

#define NUM_ARM_JOINTS 4

typedef struct
{
	uint8_t mode;
	double leftArmGripperPositionRef;
	double leftArmJointPositionRef[NUM_ARM_JOINTS];
	double leftArmCartesianPositionRef[3];
	double leftArmJointTorqueRef[NUM_ARM_JOINTS];
	double leftArmForce[3];
	double rightArmGripperPositionRef;
	double rightArmJointPositionRef[NUM_ARM_JOINTS];
	double rightArmCartesianPositionRef[3];
	double rightArmJointTorqueRef[NUM_ARM_JOINTS];
	double rightArmForce[3];
	double timeStamp;
} ARMS_CONTROL_REFERENCES_DATA_PACKET;


typedef struct
{
    double leftArmJointValues[NUM_ARM_JOINTS];        // Joint values in [rad]
    double rightArmJointValues[NUM_ARM_JOINTS];        // Joint values in [rad]
	double leftArmJointSpeed[NUM_ARM_JOINTS];		// Joint speed in [rad/s]
	double rightArmJointSpeed[NUM_ARM_JOINTS];		// Joint speed in [rad/s]
	double leftArmJointPWM[NUM_ARM_JOINTS];		    // Joint PWM in [-1, 1]
	double rightArmJointPWM[NUM_ARM_JOINTS];		// Joint PWM in [-1, 1]
	double leftTCPCartesianPosition[3];				// Tool Center Point Cartesian position in [m]
	double rightTCPCartesianPosition[3];			// Tool Center Point Cartesian position in [m]
} ARMS_STATE_PUBLISHER_DATA_PACKET;

union UINT16_BYTES {
	unsigned short value;
	char bytes[sizeof(unsigned short)];
};

union DOUBLE_BYTES {
	double value;
	char bytes[sizeof(double)];
};

struct Pose_Data{
	UINT16_BYTES id;
	DOUBLE_BYTES x;
	DOUBLE_BYTES y;
	DOUBLE_BYTES z;
	DOUBLE_BYTES qw;
	DOUBLE_BYTES qx;
	DOUBLE_BYTES qy;
	DOUBLE_BYTES qz;
};

class MODEL_BASED_CONTROLLER {
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _leftSub, _rightSub, _errorSub, _gazeboClockSub, _jointStateSub;
        ros::Publisher _PositionPub, _jDroneEffPub, _jLeftArmPosPub, _jRightArmPosPub, _jLeftArmEffPub, _jRightArmEffPub, _jCablesEffPub;
        ros::ServiceClient _setModelConfigurationClient;
        gazebo_msgs::SetModelConfiguration _configSrv;
        ros::ServiceClient _pauseGazebo;
        double _gazeboTime;
        double _jLeftArmRef, _jRightArmRef, _Error;
        double _jDronePos, _jCablesPos, _jShouldersPos, _jLeftArmPos, _jRightArmPos;
        double _jDroneVel, _jCablesVel, _jShouldersVel, _jLeftArmVel, _jRightArmVel;
    public:
        MODEL_BASED_CONTROLLER();
        void run();
        void setInitialState();                              // Set system initial state
        void gazeboClockCB(rosgraph_msgs::Clock);            // Gazebo clock callback
        void jointStateCB(sensor_msgs::JointState);          // Link state callback
        void errorStateCB(std_msgs::Float64);                // Error sub
        void leftArmRef(std_msgs::Float64);                  // Left arm sub callback
        void rightArmRef(std_msgs::Float64);                 // Right arm sub callback
        void unpack_message(char*, Pose_Data*);
        
        void* armsControlReferencesThreadFunction();
        void* armsControlReferences();
        void receiveCables();                                // Receiving Loop
        
        void activeControl(Eigen::VectorXd&, Eigen::VectorXd, Eigen::VectorXd); 
        void quadActiveControl();                            // Control Loop
        void noncollocatedPFBLControl();                     // Control Loop
        
        void Bfun(Eigen::MatrixXd&, Eigen::VectorXd);
        void nfun(Eigen::VectorXd&, Eigen::VectorXd, Eigen::VectorXd);
        
        //static void * uavStateReceiver(void * args);
        //static void * uavReferenceSender(void * args);
};
