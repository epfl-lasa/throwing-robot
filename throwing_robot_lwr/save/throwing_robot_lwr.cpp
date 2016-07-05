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

#include "../include/throwing_robot_lwr.h"

#include <math.h>
#include "MathLib.h"
//#include "ThirdPoly.h"

using namespace std;
void throwing_robot_lwr::chatterCallback_release_position(const std_msgs::Float32MultiArray::ConstPtr& RELEASE_information)
{


    if(ready == 0)
    {
        i = 0;

        for(std::vector<float>::const_iterator it = RELEASE_information->data.begin(); it != RELEASE_information->data.end(); ++it)
        {
            if(i < KUKA_DOF)
            {
                release_joints[i] = *it;
            }
            else if(i < 2 *KUKA_DOF)
            {
                release_joints_vel[i - KUKA_DOF] = *it;
            }
            else
            {
                pre_release_joints[i - 2 * KUKA_DOF] = *it;
            }
            i++;
        }

        is_given_release = 0; // a target has been received
        is_achieve_release = 0;// but not achieved yet
        instruction_got1 = 1;
        go = 1;
        cout<<"A feasible velocity has been received"<<endl;

         cout<<"Joint_POS"<<release_joints[0]<<"  "<<release_joints[1]<<"  "<<release_joints[2]<<"  "<<release_joints[3]<<"  "<<release_joints[4]<<"  "<<release_joints[5]<<"  "<<release_joints[6]<<endl;
         cout<<"Joint_vel"<<release_joints_vel[0]<<"  "<<release_joints_vel[1]<<"  "<<release_joints_vel[2]<<"  "<<release_joints_vel[3]<<"  "<<release_joints_vel[4]<<"  "<<release_joints_vel[5]<<"  "<<release_joints_vel[6]<<endl;

         char vel_name[] = "../../../../Dropbox/LASA/My project/motion study/release_vel.txt";
         fstream vel(vel_name, std::fstream::in | std::fstream::out | std::fstream::app);
         if(!vel.is_open()) cout<<"problem file"<<endl;
         vel<<release_joints_vel<<std::endl;
         vel.close();

         char pos_name[] = "../../../../Dropbox/LASA/My project/motion study/release_pos.txt";
         fstream pos(pos_name, std::fstream::in | std::fstream::out | std::fstream::app);
         pos<<release_joints<<std::endl;
         pos.close();

         char pre_pos_name[] = "../../../../Dropbox/LASA/My project/motion study/release_pre_pos.txt";
         fstream pre_pos(pre_pos_name, std::fstream::in | std::fstream::out | std::fstream::app);
         pre_pos<<pre_release_joints<<std::endl;
         pre_pos.close();


        ready =1;
    }
        return;
}




throwing_robot_lwr::throwing_robot_lwr()
:RobotInterface(){
}
throwing_robot_lwr::~throwing_robot_lwr(){
}

RobotInterface::Status throwing_robot_lwr::RobotInit(){

    ros::NodeHandle *n = mRobot->InitializeROS();

    counter = 0;
    in_motion1 = 0;
    in_motion2 = 0;
    in_motion3 = 0;
    instruction_got1 = 0;
    instruction_got2 = 0;
    ready = 0;
    steady_joints.Resize(KUKA_DOF);
    steady_joints_vel.Resize(KUKA_DOF);
    release_joints.Resize(KUKA_DOF);
    release_joints_vel.Resize(KUKA_DOF);
    pre_release_joints_vel.Resize(KUKA_DOF);
    pre_release_joints.Resize(KUKA_DOF);

    steady_joints.Zero();
    steady_joints_vel.Zero();
    release_joints.Zero();
    release_joints_vel.Zero();
    pre_release_joints_vel.Zero();

    choix == 2;

//    double positionr = M_PI / 3;
//    double vitesser = 5;
//    release_joints(0) = positionr;
//    release_joints(1) = positionr;
//    release_joints(2) = positionr;
//    release_joints(3) = positionr;
//    release_joints(4) = positionr;
//    release_joints(5) = positionr;
//    release_joints(6) = positionr;


//    release_joints_vel(0) = -vitesser;
//    release_joints_vel(1) = -vitesser;
//    release_joints_vel(2) = -vitesser;
//    release_joints_vel(3) = -vitesser;
//    release_joints_vel(4) = -vitesser;
//    release_joints_vel(5) = -vitesser;
//    release_joints_vel(6) = -vitesser;



//    pre_release_joints(0) = positionr + M_PI/10;
//    pre_release_joints(1) = positionr + M_PI/10;
//    pre_release_joints(2) = positionr + M_PI/10;
//    pre_release_joints(3) = positionr + M_PI/10;
//    pre_release_joints(4) = positionr + M_PI/10;
//    pre_release_joints(5) = positionr;
//    pre_release_joints(6) = positionr;


//    is_achieve_release = 0;// but not achieved yet
//    in_motion = 0;
//    instruction_got1 = 1;

//    is_given_release = 1;

    aller=0;




    //remove files

    char vel_name_rel[] = "../../../../Dropbox/LASA/My project/motion study/release_vel.txt";
    fstream vel1(vel_name_rel, std::fstream::out );
    if(!vel1.is_open()) cout<<"problem file"<<endl;
    //vel1<<release_joints_vel<<std::endl;
    vel1.close();

    char pos_name_rel[] = "../../../../Dropbox/LASA/My project/motion study/release_pos.txt";
    fstream pos1(pos_name_rel, std::fstream::out);
    //pos1<<release_joints<<std::endl;
    pos1.close();

    char pre_pos_name[] = "../../../../Dropbox/LASA/My project/motion study/release_pre_pos.txt";
    fstream pre_pos(pre_pos_name, std::fstream::out);
    //pre_pos<<pre_release_joints<<std::endl;
    pre_pos.close();

    char vel_name[] = "../../../../Dropbox/LASA/My project/motion study/vel_actual.txt";
    fstream vel(vel_name, std::fstream::out );
    if(!vel.is_open()) cout<<"problem file"<<endl;
    //vel<<mJointVelAll<<std::endl;
    vel.close();

    char pos_name[] = "../../../../Dropbox/LASA/My project/motion study/pos_actual.txt";

    fstream pos(pos_name, std::fstream::out);
    //pos<<mJointPosAll<<std::endl;
    pos.close();


    char vel_name_des[] = "../../../../Dropbox/LASA/My project/motion study/vel_desired.txt";
    fstream vel_des(vel_name_des, std::fstream::out );
    if(!vel_des.is_open()) cout<<"problem file"<<endl;
    //vel_des<<mJointDesVel<<std::endl;
    vel_des.close();

    char pos_name_des[] = "../../../../Dropbox/LASA/My project/motion study/pos_desired.txt";

    fstream pos_des(pos_name_des, std::fstream::out );
    //pos_des<<mJointDesPos<<std::endl;
    pos_des.close();

    char pos_name_end[] = "../../../../Dropbox/LASA/My project/motion study/pos_end_effect.txt";

    fstream pos_end(pos_name_end,  std::fstream::out );
    //pos_end<<lPos<<std::endl;
    pos_end.close();


    //test should be removed after

//    char vel_name3[] = "../../../../Dropbox/LASA/My project/motion study/release_vel.txt";
//    fstream vel3(vel_name3, std::fstream::in | std::fstream::out | std::fstream::app);
//    if(!vel3.is_open()) cout<<"problem file"<<endl;
//    vel3<<release_joints_vel<<std::endl;
//    vel3.close();

//    char pos_name3[] = "../../../../Dropbox/LASA/My project/motion study/release_pos.txt";
//    fstream pos3(pos_name3, std::fstream::in | std::fstream::out | std::fstream::app);
//    pos3<<release_joints<<std::endl;
//    pos3.close();

//    char pre_pos_name3[] = "../../../../Dropbox/LASA/My project/motion study/release_pre_pos.txt";
//    fstream pre_pos3(pre_pos_name3, std::fstream::in | std::fstream::out | std::fstream::app);
//    pre_pos3<<pre_release_joints<<std::endl;
//    pre_pos3.close();

    //end test


    //release_joints = release_joints + M_PI/3; // every joint

    //release_joints_vel = release_joints_vel + 3*M_PI; // every joint
    //steady_joints_vel = steady_joints_vel + M_PI; // every joint


    TPOLY = new	ThirdPoly(KUKA_DOF);



    // resize the global variables
    mInitialJointPos.Resize(KUKA_DOF);
    mJointPos.Resize(KUKA_DOF);
    mJointKinematics.Resize(KUKA_DOF);
    mJointDesPos.Resize(KUKA_DOF);
    mJointTargetPos.Resize(KUKA_DOF);
    mJointDesVel.Resize(KUKA_DOF);
    mJointDesResidualVel.Resize(KUKA_DOF);
    mJointTorque.Resize(KUKA_DOF);
    lJointWeight.Resize(KUKA_DOF);
    lPos.Resize(3);
    lJoints.Resize(KUKA_DOF);
    lJacobianDirY.Resize(3,KUKA_DOF);
    lJacobianDirZ.Resize(3,KUKA_DOF);
    lTargetPos.Resize(3);
    lTDirection.Resize(3);

    mJointPosAll.Resize(KUKA_DOF+FINGER_DOF);
    mJointTorqueAll.Resize(KUKA_DOF+FINGER_DOF);

    // for inverse kinematics
    mJointVelLimitsUp.Resize(KUKA_DOF);
    mJointVelLimitsDn.Resize(KUKA_DOF);
    mJointVelocityLimitsDT.Resize(KUKA_DOF);
    mTargetVelocity.Resize(IK_CONSTRAINTS);

    // initialize sensor group
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
    mSensorsGroup.ReadSensors();

    // Kinematic Chain for real robot
    mEndEffectorId = mRobot->GetLinksCount()-1;
    mKinematicChain.SetRobot(mRobot);
    mKinematicChain.Create(0,0,mEndEffectorId);

    // kinematic chain for virtual robot
    mSKinematicChain = new sKinematics(KUKA_DOF, _dt);

    // you should call sKin->setHD function lDofRobot times
    // @index               : starting from 0 to (num-1)
    // @a, d, alpha, theta0 : D-H Parameters
    // @min, max            : joint limits
    // @maxVel              : maximum joint speed

    mSKinematicChain->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD( -85.), DEG2RAD( 85.), DEG2RAD(98.0)*0.90);
    mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(98.0)*0.90);
    mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-100.), DEG2RAD(100.), DEG2RAD(100.0)*0.90);
    mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(130.0)*0.90);
    mSKinematicChain->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-140.), DEG2RAD(140.), DEG2RAD(140.0)*0.90);
    mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(180.0)*0.90); // reduced joint angle to save the fingers
    //	mSKinematicChain->setDH(6, 0.0, 0.1260,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // for sim lab
    mSKinematicChain->setDH(6, -0.05,   0.2290,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // with Alegro Hand
    mSKinematicChain->readyForKinematics();
    // T0 is a transformation matrix from global basement to base coordinate of 0th links
    // T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
    double T0[4][4];
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            T0[i][j] = 0.0;

    T0[0][0] = 1;
    T0[1][1] = 1;
    T0[2][2] = 1;
    T0[3][3] = 1;
    mSKinematicChain->setT0(T0);


    MathLib::Matrix3 mTF;
    double TF[4][4];
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            TF[i][j] = 0.0;
    TF[3][3] = 1;
    mTF = Matrix3::SRotationY(M_PI/4.0);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            TF[i][j] = mTF(i,j);

    //TF[1][3] = 0.03;

    // ready for kinematics
    mSKinematicChain->readyForKinematics();

    mSKinematicChain->setTF(TF);

    // variable for ik
    mJacobian3.Resize(3,KUKA_DOF);
    mJacobian6.Resize(6,KUKA_DOF);
    mJacobian9.Resize(9,KUKA_DOF);

    // Inverse kinematics
    mIKSolver.SetSizes(KUKA_DOF);  // Dof counts
    mIKSolver.AddSolverItem(IK_CONSTRAINTS);
    mIKSolver.SetVerbose(false);                // No comments
    mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
    mIKSolver.Enable(true,0);                   // Enable first solver
    mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

    lJointWeight(0) = 0.35;
    lJointWeight(1) = 0.35;
    lJointWeight(2) = 0.35;
    lJointWeight(3) = 0.35;
    lJointWeight(4) = 1.0;
    lJointWeight(5) = 1.0;
    lJointWeight(6) = 1.0;

    mIKSolver.SetDofsWeights(lJointWeight);

    Vector lMaxVel(KUKA_DOF);
    mSKinematicChain->getMaxVel(lMaxVel.Array());
    mJointVelocityLimitsDT = lMaxVel*_dt;

    mJobJoints.Resize(KUKA_DOF);
    mJobJoints.Set(cJob, KUKA_DOF);
    mJointWeights.Resize(KUKA_DOF);


    //trying something
    mSKinematicChain->getJacobianDirection(AXIS_Z, lJacobianDirZ);
    // Set maximum joint velocity
    for(int i=0;i<KUKA_DOF;i++){
        mJointVelLimitsDn(i) = -mSKinematicChain->getMaxVel(i);
        mJointVelLimitsUp(i) =  mSKinematicChain->getMaxVel(i);
    }
    mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);

    //mSKinematicChain->setJoints(Q_end.Array());
    //mSKinematicChain->getEndPos(lPos_final.Array());




    sub_release_position = n->subscribe("joint_velocities", 3, & throwing_robot_lwr::chatterCallback_release_position,this);




    AddConsoleCommand("test");
    AddConsoleCommand("job");
    mPlanner =NONE;
    return STATUS_OK;
}
RobotInterface::Status throwing_robot_lwr::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status throwing_robot_lwr::RobotStart(){
    ros::spinOnce();

    return STATUS_OK;
}    
RobotInterface::Status throwing_robot_lwr::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status throwing_robot_lwr::RobotUpdate(){

    ros::spinOnce();

    mSKinematicChain->setJoints(mJointKinematics.Array());
    mSKinematicChain->getEndPos(lPos.Array());
    mSKinematicChain->getEndDirAxis(AXIS_X, lDirX.Array());
    mSKinematicChain->getEndDirAxis(AXIS_Y, lDirY.Array());
    mSKinematicChain->getEndDirAxis(AXIS_Z, lDirZ.Array());
    mSKinematicChain->getJoints(mJointDesPos.Array());

    switch(mCommand){
    case COMMAND_TEST :
        mPlanner = PLANNER_CARTESIAN;
        break;
    case COMMAND_JOB:
        mPlanner = PLANNER_JOINT;
    }

    switch(mPlanner){

    case PLANNER_CARTESIAN:


        if(choix ==1)
        {

        if(instruction_got1==1 && in_motion == 0 )
        {

            TPOLY->SetConstraints(mJointPosAll,mJointVelAll,pre_release_joints,pre_release_joints_vel,3.0); //compute the time accordingly to the robot constraints

          in_motion = 1;
          cout<<"1"<<endl;
          counter = 0;


        }

        if(fabs(counter-3) <0.00001 && in_motion == 1 && instruction_got1 == 1 )
        {

            in_motion = 0;
            instruction_got1 = 0;
            is_given_release = 1;
            cout<<"2"<<endl;


                    TPOLY->SetConstraints(mJointPosAll,mJointVelAll,release_joints,release_joints_vel,3.0); //compute the time accordingly to the robot constraints
                    is_given_release = 0;
                    in_motion = 1;
                    counter = 0;
                    not_begining = 0;

                    aller = 1;
                    cout<<"3"<<endl;

                }



                if(fabs(counter-3) <0.00001 && in_motion == 1 && aller == 1 )
                {
                    counter = 0;

                  TPOLY->SetConstraints(mJointPosAll,release_joints_vel,steady_joints,steady_joints_vel,5.0); //compute the time accordingly to the robot constraints

                    //cout<<"counter = "<<counter<<endl;
                    cout<<"act vel = "<<mJointVelAll<<endl;
                    //cout<<"des vel = "<<mJointDesVel<<endl;
                    cout<<"act pos = "<<mJointPosAll<<endl;
                    cout<<"des pos = "<<mJointDesPos<<endl;
                    //TPOLY->SetConstraints(release_joints,release_joints_vel,steady_joints,steady_joints_vel,5.0); //compute the time accordingly to the robot constraints

                    counter = 0;
                    not_begining = 1;
                    cout<<"release point achieved"<<endl;
                    aller = 0;
                    cout<<"4"<<endl;
                mSKinematicChain->getEndPos(lPos.Array());
                ROS_INFO_STREAM("POS : "<<lPos);

                }


                //if(counter > 4.9 && counter < 5.1 ) cout<<"counter = "<<counter<<endl;

                if(fabs(counter-5) <0.00001 && not_begining == 1)
                {
                    cout<<":/"<<endl;
                    in_motion = 0;
                    not_begining = 0;

                    counter = 0;
                    ready = 0;
                    is_given_release = 0;
                    //instruction_got1 = 1;
                    cout<<"5"<<endl;

                }

                if(in_motion == 1)
                {

                    TPOLY->Get(counter,mJointDesPos);

                    counter = counter + _dt ;



                }


                mSKinematicChain->getEndPos(lPos.Array());


        //break;
        char vel_name[] = "../../../../Dropbox/LASA/My project/motion study/vel_actual.txt";
        fstream vel(vel_name, std::fstream::in | std::fstream::out | std::fstream::app);
        if(!vel.is_open()) cout<<"problem file"<<endl;
        vel<<mJointVelAll<<std::endl;
        vel.close();

        char pos_name[] = "../../../../Dropbox/LASA/My project/motion study/pos_actual.txt";

        fstream pos(pos_name, std::fstream::in | std::fstream::out | std::fstream::app);
        pos<<mJointPosAll<<std::endl;
        pos.close();


        char vel_name_des[] = "../../../../Dropbox/LASA/My project/motion study/vel_desired.txt";
        fstream vel_des(vel_name_des, std::fstream::in | std::fstream::out | std::fstream::app);
        if(!vel_des.is_open()) cout<<"problem file"<<endl;
        vel_des<<mJointDesVel<<std::endl;
        vel_des.close();

        char pos_name_des[] = "../../../../Dropbox/LASA/My project/motion study/pos_desired.txt";

        fstream pos_des(pos_name_des, std::fstream::in | std::fstream::out | std::fstream::app);
        pos_des<<mJointDesPos<<std::endl;
        pos_des.close();

        char pos_name_end[] = "../../../../Dropbox/LASA/My project/motion study/pos_end_effect.txt";

        fstream pos_end(pos_name_end, std::fstream::in | std::fstream::out | std::fstream::app);
        pos_end<<lPos<<std::endl;
        pos_end.close();


        }
        else if(choix == 2)
        {

            mSKinematicChain->getEndPos(lPos.Array());

            if(instruction_got1==1 && in_motion == 0 )
            {

                TPOLY->SetConstraints(mJointPosAll,mJointVelAll,pre_release_joints,pre_release_joints_vel,3.0); //compute the time accordingly to the robot constraints

              in_motion = 1;
              cout<<"1"<<endl;
              counter = 0;


            }

            if(fabs(counter-3) <0.00001 && in_motion == 1 && instruction_got1 == 1 )
            {

                in_motion = 0;
                instruction_got1 = 0;
                is_given_release = 1;
                cout<<"2"<<endl;


                        TPOLY->SetConstraints(mJointPosAll,mJointVelAll,release_joints,release_joints_vel,3.0); //compute the time accordingly to the robot constraints
                        is_given_release = 0;
                        in_motion = 1;
                        counter = 0;
                        not_begining = 0;

                        aller = 1;
                        cout<<"3"<<endl;

                    }



                    if(fabs(counter-3) <0.00001 && in_motion == 1 && aller == 1 )
                    {
                        counter = 0;

                      TPOLY->SetConstraints(mJointPosAll,release_joints_vel,steady_joints,steady_joints_vel,5.0); //compute the time accordingly to the robot constraints

                        //cout<<"counter = "<<counter<<endl;
                        cout<<"act vel = "<<mJointVelAll<<endl;
                        //cout<<"des vel = "<<mJointDesVel<<endl;
                        cout<<"act pos = "<<mJointPosAll<<endl;
                        cout<<"des pos = "<<mJointDesPos<<endl;
                        //TPOLY->SetConstraints(release_joints,release_joints_vel,steady_joints,steady_joints_vel,5.0); //compute the time accordingly to the robot constraints

                        counter = 0;
                        not_begining = 1;
                        cout<<"release point achieved"<<endl;
                        aller = 0;
                        cout<<"4"<<endl;
                    mSKinematicChain->getEndPos(lPos.Array());
                    ROS_INFO_STREAM("POS : "<<lPos);

                    }


                    //if(counter > 4.9 && counter < 5.1 ) cout<<"counter = "<<counter<<endl;

                    if(fabs(counter-5) <0.00001 && not_begining == 1)
                    {
                        cout<<":/"<<endl;
                        in_motion = 0;
                        not_begining = 0;

                        counter = 0;
                        ready = 0;
                        is_given_release = 0;
                        //instruction_got1 = 1;
                        cout<<"5"<<endl;

                    }

                    if(in_motion == 1)
                    {

                        TPOLY->Get(counter,mJointDesPos);

                        counter = counter + _dt ;



                    }


                    mSKinematicChain->getEndPos(lPos.Array());


            //break;
            char vel_name[] = "../../../../Dropbox/LASA/My project/motion study/vel_actual.txt";
            fstream vel(vel_name, std::fstream::in | std::fstream::out | std::fstream::app);
            if(!vel.is_open()) cout<<"problem file"<<endl;
            vel<<mJointVelAll<<std::endl;
            vel.close();

            char pos_name[] = "../../../../Dropbox/LASA/My project/motion study/pos_actual.txt";

            fstream pos(pos_name, std::fstream::in | std::fstream::out | std::fstream::app);
            pos<<mJointPosAll<<std::endl;
            pos.close();


            char vel_name_des[] = "../../../../Dropbox/LASA/My project/motion study/vel_desired.txt";
            fstream vel_des(vel_name_des, std::fstream::in | std::fstream::out | std::fstream::app);
            if(!vel_des.is_open()) cout<<"problem file"<<endl;
            vel_des<<mJointDesVel<<std::endl;
            vel_des.close();

            char pos_name_des[] = "../../../../Dropbox/LASA/My project/motion study/pos_desired.txt";

            fstream pos_des(pos_name_des, std::fstream::in | std::fstream::out | std::fstream::app);
            pos_des<<mJointDesPos<<std::endl;
            pos_des.close();

            char pos_name_end[] = "../../../../Dropbox/LASA/My project/motion study/pos_end_effect.txt";

            fstream pos_end(pos_name_end, std::fstream::in | std::fstream::out | std::fstream::app);
            pos_end<<lPos<<std::endl;
            pos_end.close();
        }

    }
    mSKinematicChain->setJoints(mJointDesPos.Array());
    mJointKinematics.Set(mJointDesPos);



    return STATUS_OK;
}
RobotInterface::Status throwing_robot_lwr::RobotUpdateCore(){

    ros::spinOnce();

    mSensorsGroup.ReadSensors();

    //mJointAccAll    = mSensorsGroup.GetJointAccelerations();
    //mJointVelAll    = mSensorsGroup.GetJointVelocities();

    mJointPosAll_old = mJointPosAll;
    mJointPosAll    = mSensorsGroup.GetJointAngles();
    mJointVelAll = (mJointPosAll - mJointPosAll_old)/_dt;
    //mJointPosAll.Print("mJointPosAll");
    for(int i=0; i<KUKA_DOF; i++) mJointTargetPos(i)= mJointPosAll(i);
//	mJointTorqueAll = mSensorsGroup.GetJointTorques();
//	for(int i=0; i<KUKA_DOF; i++) mJointTorque(i)= mJointTorqueAll(i);

    if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
            mRobot->SetControlMode(Robot::CTRLMODE_POSITION);



    mActuatorsGroup.SetJointAngles(mJointDesPos);
    //mActuatorsGroup.SetJointVelocities(mJointDesVel);
    mActuatorsGroup.WriteActuators();
    mKinematicChain.Update();

    return STATUS_OK;
}
int throwing_robot_lwr::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	cout<<"Write your command"<<endl;

    if(cmd=="job"){
        cout<<"job 2 2"<<endl;
        mCommand = COMMAND_JOB;

	}
	else if(cmd=="test"){
        cout<<"tets  1 1"<<endl;
        mCommand = COMMAND_TEST;
	}

    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    throwing_robot_lwr* create(){return new throwing_robot_lwr();}
    void destroy(throwing_robot_lwr* module){delete module;}
}

