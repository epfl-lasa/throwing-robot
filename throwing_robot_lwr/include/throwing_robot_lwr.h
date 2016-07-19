/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef throwing_robot_lwr_H_
#define throwing_robot_lwr_H_

#include "RobotLib/RobotInterface.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include "ThirdPoly.h"


#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "sKinematics.h"
#include <iomanip>
#include <iostream>
#include <fstream>
#include <std_msgs/Int8.h>
#include <boost/date_time.hpp>

#define KUKA_DOF 7
#define FINGER_DOF 0
#define IK_CONSTRAINTS 9
#define _dt (1.0/500.)
//#define _dt (0.11)

double cJob[]  = {0.0, -PI/4.0, 0.0, -PI/2.0, 0.0, -PI/4.0, 0.0};
enum ENUM_COMMAND{COMMAND_JOB,COMMAND_TEST,COMMAND_POS};
enum ENUM_PLANNER{PLANNER_CARTESIAN, PLANNER_JOINT,PLANNER_POS,NONE};
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};

//class ThirdPoly;// TPOLY(KUKA_DOF);
//ThirdPoly TPOLY(KUKA_DOF);


class throwing_robot_lwr : public RobotInterface
{
public:
            throwing_robot_lwr();
    virtual ~throwing_robot_lwr();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

protected:
    void                        chatterCallback_release_position(const std_msgs::Float32MultiArray::ConstPtr& RELEASE_information);

sKinematics                 *mSKinematicChain;

	RevoluteJointSensorGroup    mSensorsGroup;
	RevoluteJointActuatorGroup  mActuatorsGroup;
	KinematicChain              mKinematicChain;

	IKGroupSolver               mIKSolver;

	int                         mEndEffectorId;
	Vector                      mInitialJointPos;
	Vector                      mJointKinematics;
	Vector                      mJointPos;
	Vector                      mJointPosAll;
    Vector                      mJointPosAll_old;
    Vector                      mJointDesPos_old;
    Vector                      mJointVelAll;
    Vector                      mJointAccAll;

	Vector                      mJointDesPos;
	Vector                      mJointTargetPos;

	Vector                      mJointDesVel;
	Vector                      mJointDesResidualVel;

	Vector                      mJointTorque;
	Vector                      mJointTorqueAll;

	Vector                      mJointVelLimitsUp;
	Vector                      mJointVelLimitsDn;
	Vector       				mJointVelocityLimitsDT;
	Vector 						mTargetVelocity; // for inverse kinematics input
	Vector 						lJointWeight;
	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;
	Vector 						lPos;
	Vector3 					lDirX, lDirY, lDirZ;
	Vector 						lJoints;
	Matrix 						lJacobianDirY;
	Matrix 						lJacobianDirZ;
	// Jacobian Variables
	Matrix 						mJacobian3;
	Matrix 						mJacobian6;
	Matrix 						mJacobian9;
	Vector 						mJobJoints;
	Vector						mreleaseJoints;
	Vector 						mJointWeights;
	Vector					   	mreleaseJoints2;
	Vector 						lTargetPos;
	Vector						lTDirection;
	Vector3 					lTargetDirX;
	Vector3						lTargetDirY;
	Vector3 					lTargetDirZ;
	Vector 						steady_joints;
	Vector 						steady_joints_vel;
	Vector 						release_joints;
    Vector 						release_joints_vel;
    Vector                      pre_release_joints;
    Vector                      pre_release_joints_vel;
    Vector                      release_pos;
    Vector                      pre_release_pos;


    int 						in_motion1;
    int 						in_motion2;
    int 						in_motion3;
    int                         instruction_got1;
    int                         instruction_got2;
    int                         			not_begining;
	int 						is_given_release;
	int 						is_achieve_release;
    int 						in_motion;
	int                         			i;
    double 						counter;
    ThirdPoly                   *TPOLY;
    ros::Subscriber             sub_release_position;
    ros::Publisher              open_hand;
    int aller;
    int go;
    int                         ready;
    int                         choix;
    int                         back;
    int                         hhh;
    int tr;
    int                         waitd;
    int                         phase;
    int                         time1;
    int                         time2;
    int                         time3;
    int                         time4;
    int                         time5;
    int                         block;
    std_msgs::Int8              msg;
    boost::posix_time::ptime    start_time;
    boost::posix_time::time_duration diff;
};



#endif 
