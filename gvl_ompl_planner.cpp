// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*
#include <iostream>
using namespace std;
#include <signal.h>

#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <unistd.h> 
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <ompl/geometric/PathSimplifier.h>

#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <memory>

#include <Python.h>
#include <stdlib.h>
#include <vector>
namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
using namespace KDL;
using namespace std;
// initial quaternion 0.49996,0.86605,0.00010683,0



std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr;
enum States { READY, FIND, GRASP, MOVE1,MOVE2,UNGRASP };

void doTaskPlanning(double* goal_values){


    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(7));
    //We then set the bounds for the R3 component of this state space:
    ob::RealVectorBounds bounds(7);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
 
    space->setBounds(bounds);
    //Create an instance of ompl::base::SpaceInformation for the state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));

    //Set the state validity checker
       std::cout << "-------------------------------------------------" << std::endl;
    og::PathSimplifier simp(si);

    si->setStateValidityChecker(my_class_ptr->getptr());
    si->setMotionValidator(my_class_ptr->getptr());
    si->setup();
    KDL::Tree my_tree;
    KDL::Chain my_chain;


   if (!kdl_parser::treeFromFile("panda_coarse/panda_7link.urdf", my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
   }

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl);
    LOGGING_INFO(Gpu_voxels, "\n\nKDL Chain load : "<<my_tree.getChain("panda_link0","panda_link8",my_chain) <<"\n"<< endl);
  
    std::cout<<my_chain.getNrOfJoints()<<std::endl;


    KDL::JntArray q1(my_chain.getNrOfJoints());
    KDL::JntArray q_init(my_chain.getNrOfJoints());
  
    std::cout<<my_chain.getNrOfJoints()<<std::endl;

    double start_values[7];
    my_class_ptr->getJointStates(start_values);
    q_init(0) = 0;
    q_init(1) =  -0.7850857777;
    q_init(2) = 0;
    q_init(3) = -2.3555949;
    q_init(4) = 0;
    q_init(5) = 1.57091693296;
    q_init(6) = 1.57091693296;
    KDL::JntArray q_min(my_chain.getNrOfJoints()),q_max(my_chain.getNrOfJoints());
    q_min(0) = -2.8973;
    q_min(1) = -1.7628;
    q_min(2) = -2.8973;
    q_min(3) = -3.0718;
    q_min(4) = -2.8973;
    q_min(5) = -0.0175;
    q_min(6) = -2.8973;

    q_max(0) = 2.8973;
    q_max(1) = 1.7628;
    q_max(2) = 2.8973;
    q_max(3) = -0.0698;
    q_max(4) = 2.8973;
    q_max(5) = 3.7525;
    q_max(6) = 2.8973;



    KDL::Frame cart_pos;
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    fk_solver.JntToCart(q_init, cart_pos);
    cout<<cart_pos<<endl;
    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.01);
  	KDL::Vector pos = KDL::Vector(0.05,-0.1865,1.20);
  	
    //KDL::Frame goal_pose( KDL::Rotation::RPY(goal_values[0],goal_values[1],goal_values[2]),KDL::Vector(goal_values[3],goal_values[4],goal_values[5]));
    KDL::Frame goal_pose( KDL::Rotation::Quaternion(goal_values[0],goal_values[1],goal_values[2],goal_values[3]),KDL::Vector(goal_values[4],goal_values[5],goal_values[6]));

    bool ret = iksolver1.CartToJnt(q_init,goal_pose,q1);
    std::cout<<"ik ret : "<<ret<<std::endl;
    std::cout<<"ik q : "<<q1(0)<<","<<q1(1)<<","<<q1(2)<<","<<q1(3)<<","<<q1(4)<<","<<q1(5)<<","<<q1(6)<<std::endl;
    
    ob::ScopedState<> start(space);


    start[0] = double(q_init(0));
    start[1] = double(q_init(1));
    start[2] = double(q_init(2));
    start[3] = double(q_init(3));
    start[4] = double(q_init(4));
    start[5] = double(q_init(5));
    start[6] = double(q_init(6));

   ob::ScopedState<> goal(space);
    goal[0] = double(q1(0));
    goal[1] =  double(q1(1));
    goal[2] =  double(q1(2));
    goal[3] = double(q1(3));
    goal[4] =  double(q1(4));
    goal[5] = double(q1(5));
    goal[6] = double(q1(6));
    

	my_class_ptr->insertStartAndGoal(start, goal);
    my_class_ptr->doVis();

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    auto planner(std::make_shared<og::LBKPIECE1>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    int succs = 0;

    std::cout << "Waiting for Viz. Press Key if ready!" << std::endl;
    ob::PathPtr path ;
     while(succs<1)
    {
        try{
                my_class_ptr->moveObstacle();
                planner->clear();
                ob::PlannerStatus  solved = planner->ob::Planner::solve(200.0);
                PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");


                if (solved)
                {
                    ++succs;
                    path = pdef->getSolutionPath();
                    std::cout << "Found solution:" << std::endl;
                    path->print(std::cout);
                    simp.simplifyMax(*(path->as<og::PathGeometric>()));

                }else{
                    std::cout << "No solution could be found" << std::endl;
                }

                PERF_MON_SUMMARY_PREFIX_INFO("planning");
                std::cout << "END OMPL" << std::endl;
                my_class_ptr->doVis();
               }
        catch(int expn){
        }


    }
    PERF_MON_ADD_STATIC_DATA_P("Number of Planning Successes", succs, "planning");

    PERF_MON_SUMMARY_PREFIX_INFO("planning");

    // keep the visualization running:
    og::PathGeometric* solution= path->as<og::PathGeometric>();
    solution->interpolate();

    int step_count = solution->getStateCount();
    std::vector<std::array<double,7>> q_list;
    q_list.clear();
    for(int i=0;i<step_count;i++){
        const double *values = solution->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
        double *temp_values = (double*)values;
        std::array<double,7> temp_joints_value={{temp_values[0],temp_values[1],temp_values[2],temp_values[3],temp_values[4],temp_values[5],temp_values[6]}};
        q_list.push_back(temp_joints_value);
        my_class_ptr->rosPublishJointStates(temp_values);
    	my_class_ptr->visualizeRobot(values);
	
     }
    my_class_ptr->rosPublishJointTrajectory(q_list);
    q_list.clear();

}

void doVis(){

	my_class_ptr->doVis();
	my_class_ptr->moveObstacle();

}
int main(int argc, char **argv)
{


 
signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

    icl_core::logging::initialize(argc, argv);

    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(7));
    //We then set the bounds for the R3 component of this state space:
    ob::RealVectorBounds bounds(7);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
 	bounds.setLow(0,-2.8973);
    bounds.setHigh(0,2.9671);

    bounds.setLow(1,-1.7628);
    bounds.setHigh(1,1.7628);

    bounds.setLow(2,-2.8973);
    bounds.setHigh(2,2.8973);

    bounds.setLow(3,-3.0718);
    bounds.setHigh(3,-0.0698);

    bounds.setLow(4,-2.8973);
    bounds.setHigh(4,2.8973);

    bounds.setLow(5,-0.0175);
    bounds.setHigh(5,3.7525);

    bounds.setLow(6,-2.8973);
    bounds.setHigh(6,2.8973);

 
    space->setBounds(bounds);
    //Create an instance of ompl::base::SpaceInformation for the state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    //Set the state validity checker
    std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));
    my_class_ptr->doVis();
    float roll = atof(argv[1]);
    float pitch = atof(argv[2]);
    float yaw = atof(argv[3]);
    float X = atof(argv[4]);
    float Y = atof(argv[5]);
    float Z = atof(argv[6]);
    

    my_class_ptr->setParams(roll,pitch,yaw,X,Y,Z);
    thread t1{&GvlOmplPlannerHelper::rosIter ,my_class_ptr};    
    //thread t2(jointStateCallback);
    thread t3(doVis);


    sleep(10);

   States state = READY;
   int toggle = 1;


	double task_goal_values00[7] ={0.92395,-0.38252,0,0,0.554,0.30,0.49032};
       doTaskPlanning(task_goal_values00);
	double task_goal_values11[7] ={0.92395,-0.38252,0,0,0.554,-0.30,0.49032};
       doTaskPlanning(task_goal_values11);  
       doTaskPlanning(task_goal_values00); 
       doTaskPlanning(task_goal_values11);       

	cout<<"END second Task"<<endl;

//----------------------------------------------------//
    t1.join();
    //t2.join();
    t3.join();
    return 1;
}
