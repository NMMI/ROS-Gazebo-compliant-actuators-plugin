#include <compliant_plugin/compliant_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <compliant_plugin/state_info.h>

using namespace gazebo;
using namespace std;

/*
  This code implements the methods of the CompliantActuatorPlugin class.
  For more details please refer to the article "An Open-Source ROS-Gazebo Toolbox for Simulating Robots With Compliant Actuators"
  by Mengacci et. al., published in Frontiers in Robotics and AI in 2021.
*/

// Saturation for input torque
double CompliantActuatorPlugin::saturate(double val, double max_val)
{
    if (val > max_val) {
        val = max_val;
    }
    if (val < -max_val) {
        val = -max_val;
    }
    return val;
}

void CompliantActuatorPlugin::DCMotor::OnUpdate(const double & dT, double & tauMot, const double & tauLoad){

    tauMot = saturate(tauMot, tauMax);

    const double K = 1e3;

    if  (pos - w < -tauFric/K){
        w = pos + tauFric/K;
    }
    else if (pos - w > tauFric/K){
        w = pos - tauFric/K;
    }
    // else w = w;

    double tauFricNow = K *(pos - w);

    effort = tauMot;
    double acc = (effort - tauLoad - tauFricNow - D*vel)/ J;

    vel = vel + acc * dT;

    pos = pos + vel * dT;

    if(pos < minPos){
        pos = minPos;
        vel = 0;
        effort = std::numeric_limits<double>::quiet_NaN();
    }
    else if(pos > maxPos){
        pos = maxPos;
        vel = 0;
        effort = std::numeric_limits<double>::quiet_NaN();
    }

}


void CompliantActuatorPlugin::PIDController::OnUpdate(const double & dT, const double & pos, const double & vel){
    double e = posRef - pos;
    double de = velRef - vel;
    errInt += I * e * dT;

    effort = errInt + P * e + D * de;
}


void CompliantActuatorPlugin::tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out){

    std::string warning_msg = "Make sure to define the virtual function for the CompliantActuatorPlugin!";
    ROS_WARN_STREAM(warning_msg);

};


void CompliantActuatorPlugin::eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out){

    std::string warning_msg = "Make sure to define the virtual function for the CompliantActuatorPlugin!";
    ROS_WARN_STREAM(warning_msg);

}; 

void CompliantActuatorPlugin::InitParams(sdf::ElementPtr _sdf){

    std::string warning_msg = "Make sure to define the virtual function for the CompliantActuatorPlugin!";
    ROS_WARN_STREAM(warning_msg);
}

// Subscriber callbacks references
void CompliantActuatorPlugin::getRef1_callback(const std_msgs::Float64& val_1){
	
    ref_1 = val_1.data;
}
void CompliantActuatorPlugin::getRef2_callback(const std_msgs::Float64& val_2){
    
    ref_2 = val_2.data;
}

// Subscriber callback for external torque
void CompliantActuatorPlugin::getExtTau_callback(const std_msgs::Float64& e_tau){
    
    ext_tau = e_tau.data;
}

// DEBUG PRINT FUNCTION 
void printForDebug(double tauEl_1, double tauEl_2, double tEl_L, double mot_1, double mot_2, double qL, double ref_1, double ref_2)
{
    std::cout << "T1 " << tauEl_1 << "\t";
    std::cout << "T2 " << tauEl_2 << "\t";
    std::cout << "TL " << tEl_L << "\t";

    std::cout << "M1 " << mot_1 << "\t";
    std::cout << "M2 " << mot_2 << "\t";

    std::cout << "Q " << qL << "\t";
    std::cout << "R1 " << ref_1 << "\t";
    std::cout << "R2 " << ref_2 << "\n";
}

// OPERATION MODE = 6
void CompliantActuatorPlugin::OnUpdatePIDEqPres(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    //calculate motor references
    double qR1, qR2;
    eqPres2Refs(ref_1, ref_2, qR1, qR2);

    // update controllers
    ctrl_1.posRef = qR1;
    ctrl_2.posRef = qR2;
    ctrl_1.OnUpdate(dT, mot_1.pos, mot_1.vel);
    ctrl_2.OnUpdate(dT, mot_2.pos, mot_2.vel);

    // compute old elastic torque
    double tauEl_1_old, tauEl_2_old, tEl_L_old, sigmaEl_L_old;
    tauElastic(mot_1.pos, mot_2.pos, qL, tauEl_1_old, tauEl_2_old, tEl_L_old, sigmaEl_L_old);

    //update motors
    mot_1.OnUpdate(dT, ctrl_1.effort, tauEl_1_old);
    mot_2.OnUpdate(dT, ctrl_2.effort, tauEl_2_old);

    // compute new elastic torque
    double tauEl_1, tauEl_2, tEl_L, sigmaEl_L;
    tauElastic(mot_1.pos, mot_2.pos, qL, tauEl_1, tauEl_2, tEl_L, sigmaEl_L);

    // update joint effort
    joint->SetForce(0, tEl_L);

    // publish relevant topics
    CompliantActuatorPlugin::Publish(tauEl_1, tauEl_2, tEl_L, sigmaEl_L, qL, dqL, ref_1, ref_2 );
    CompliantActuatorPlugin::PublishMotors(mot_1.pos, mot_1.vel, mot_1.effort, mot_2.pos, mot_2.vel, mot_2.effort);

    // update timer
    oldTime = t;

}

// OPERATION MODE = 5
void CompliantActuatorPlugin::OnUpdatePIDRefs(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    // update controllers
    ctrl_1.posRef = ref_1;
    ctrl_2.posRef = ref_2;
    ctrl_1.OnUpdate(dT, mot_1.pos, mot_1.vel);
    ctrl_2.OnUpdate(dT, mot_2.pos, mot_2.vel);

    // compute old elastic torque
    double tauEl_1_old, tauEl_2_old, tEl_L_old, sigmaEl_L_old;
    tauElastic(mot_1.pos, mot_2.pos, qL, tauEl_1_old, tauEl_2_old, tEl_L_old, sigmaEl_L_old);

    //update motors
    mot_1.OnUpdate(dT, ctrl_1.effort, tauEl_1_old);
    mot_2.OnUpdate(dT, ctrl_2.effort, tauEl_2_old);

    // compute new elastic torque
    double tauEl_1, tauEl_2, tEl_L, sigmaEl_L;
    tauElastic(mot_1.pos, mot_2.pos, qL, tauEl_1, tauEl_2, tEl_L, sigmaEl_L);

    // update joint effort
    joint->SetForce(0, tEl_L);

    // publish relevant topics
    CompliantActuatorPlugin::Publish(tauEl_1, tauEl_2, tEl_L, sigmaEl_L, qL, dqL, ref_1, ref_2 );
    CompliantActuatorPlugin::PublishMotors(mot_1.pos, mot_1.vel, mot_1.effort, mot_2.pos, mot_2.vel, mot_2.effort);

    // update timer
    oldTime = t;

}

// OPERATION MODE = 4
void CompliantActuatorPlugin::OnUpdateMotorTorques(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    // compute elastic torque
    double tauEl_1, tauEl_2, tEl_L, sigmaEl_L;
    tauElastic(mot_1.pos, mot_2.pos, qL, tauEl_1, tauEl_2, tEl_L, sigmaEl_L);

    //update motors
    mot_1.OnUpdate(dT, ref_1, tauEl_1);
    mot_2.OnUpdate(dT, ref_2, tauEl_2);

    // DEBUG
    // printForDebug(mot_1.effort, mot_2.effort, tEl_L, mot_1.pos, mot_2.pos, qL, ref_1, ref_2);
    // END_DEBUG
    
    // update joint effort
    joint->SetForce(0, tEl_L);

    // publish relevant topics
    CompliantActuatorPlugin::Publish(tauEl_1, tauEl_2, tEl_L, sigmaEl_L, qL, dqL, ref_1, ref_2);
    CompliantActuatorPlugin::PublishMotors(mot_1.pos, mot_1.vel, mot_1.effort, mot_2.pos, mot_2.vel, mot_2.effort);

    // update timer
    oldTime = t;

}

// OPERATION MODE = 3
void CompliantActuatorPlugin::OnUpdateSpringEqPres(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    //calculate motor references
    double qR1, qR2;
    eqPres2Refs(ref_1, ref_2, qR1, qR2);

    // compute new elastic torque
    double tauEl_1, tauEl_2, tEl_L, sigmaEl_L;
    tauElastic(qR1, qR2, qL, tauEl_1, tauEl_2, tEl_L, sigmaEl_L);

    // update joint effort
    joint->SetForce(0, tEl_L);

    // publish relevant topics
    CompliantActuatorPlugin::Publish(tauEl_1, tauEl_2, tEl_L, sigmaEl_L, qL, dqL, ref_1, ref_2 );

    // update timer
    oldTime = t;

}

// OPERATION MODE = 2
void CompliantActuatorPlugin::OnUpdateSpringRefs(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    // compute new elastic torque
    double tauEl_1, tauEl_2, tEl_L, sigmaEl_L;
    tauElastic(ref_1, ref_2, qL, tauEl_1, tauEl_2, tEl_L, sigmaEl_L);

    // update joint effort
    joint->SetForce(0, tEl_L);

    // publish relevant topics
    CompliantActuatorPlugin::Publish(tauEl_1, tauEl_2, tEl_L, sigmaEl_L, qL, dqL, ref_1, ref_2 );
    
    // update timer
    oldTime = t;

}

// OPERATION MODE = 1
void CompliantActuatorPlugin::OnUpdateLinkPID(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    // update controllers
    ctrl_1.posRef = ref_1;
    ctrl_1.OnUpdate(dT, qL, dqL);

    // update joint effort
    joint->SetForce(0, ctrl_1.effort);

    // publish relevant topics
    CompliantActuatorPlugin::Publish( std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      ctrl_1.effort, std::numeric_limits<double>::quiet_NaN(),
                                      qL, dqL, ref_1, std::numeric_limits<double>::quiet_NaN() );

    // update timer
    oldTime = t;

}

// OPERATION MODE = 0
void CompliantActuatorPlugin::OnUpdateLinkTorque(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    //retrieve joint state
    const double & qL = joint->Position(0);
    const double & dqL = joint->GetVelocity(0);

    // update joint effort
    joint->SetForce(0, ref_1);

    // publish relevant topics
    CompliantActuatorPlugin::Publish( std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      ref_1, std::numeric_limits<double>::quiet_NaN(),
                                      qL, dqL, ref_1, std::numeric_limits<double>::quiet_NaN() );

    // update timer
    oldTime = t;

}


void CompliantActuatorPlugin::OnUpdate(const common::UpdateInfo & info){
    switch(operationMode){
    case(LinkTorque):               // operationMode = 0
        OnUpdateLinkTorque(info); 
        break;  
    case(LinkPID):                  // operationMode = 1
        OnUpdateLinkPID(info);
        break;
    case(SpringRefs):               // operationMode = 2
        OnUpdateSpringRefs(info);
        break;
    case(SpringEqPres):             // operationMode = 3
        OnUpdateSpringEqPres(info);
        break;
    case(MotorTorques):             // operationMode = 4
        OnUpdateMotorTorques(info);
        break;
    case(PIDRefs):                  // operationMode = 5
        OnUpdatePIDRefs(info);
        break;
    case(PIDEqPres):                // operationMode = 6
        OnUpdatePIDEqPres(info);
        break;
    }
}

void CompliantActuatorPlugin::Publish(const double & tauEl_1,
                                      const double & tauEl_2,
                                      const double & tauEl_L,
                                      const double & sigmaEl_L,
                                      const double & q,
                                      const double & dq,
                                      const double & r_1,
                                      const double & r_2 ){
  // Publish elastic torque for driving motors
  if (flag_pub_el_tau)
  {
    elastic_tau1.data = tauEl_1;
    pubEl_1.publish(elastic_tau1);
    elastic_tau2.data = tauEl_2;
    pubEl_2.publish(elastic_tau2);
  }

  // Publish whole joint state
  if (flag_pub_state)
  {
    joint_info.tau = tauEl_L;
    joint_info.stiff = sigmaEl_L;
    joint_info.q = q;
    joint_info.dq = dq;
    joint_info.ref_1 = r_1;
    joint_info.ref_2 = r_2;
    pubL_state.publish(joint_info);
  }

}

void CompliantActuatorPlugin::PublishMotors(double m1_pos, double m1_vel, double m1_eff, double m2_pos, double m2_vel, double m2_eff){
  mot_1_state.position[0] = m1_pos;
  mot_1_state.velocity[0] = m1_vel;
  mot_1_state.effort[0] = m1_eff;
  pub_mot_1_state.publish(mot_1_state);

  mot_2_state.position[0] = m2_pos;
  mot_2_state.velocity[0] = m2_vel;
  mot_2_state.effort[0] = m2_eff;
  pub_mot_2_state.publish(mot_2_state);
}


void CompliantActuatorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "Compliant_Actuator_Plugin");

  model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve namespace and control mode from a configuration file
  ros::param::get("namespace", ns_name);
  ros::param::get("T_sample", T_sample);

  // Initialize motor variables
  mot_1_state.position.resize(1);
  mot_1_state.velocity.resize(1);
  mot_1_state.effort.resize(1);
  mot_2_state.position.resize(1);
  mot_2_state.velocity.resize(1);
  mot_2_state.effort.resize(1);

  // Retrieve joint identifier and control mode from urdf tags
  joint_name =_sdf->GetElement("joint")->Get<string>();
  //act_type =_sdf->GetElement("actuator_type")->Get<string>();
  operationMode = (OperationModes) _sdf->GetElement("operation_mode")->Get<int>();
  flag_pub_el_tau =_sdf->GetElement("pub_eltau")->Get<bool>();
  flag_pub_state =_sdf->GetElement("pub_state")->Get<bool>();
  flag_sub_ext_tau =_sdf->GetElement("sub_ext_tau")->Get<bool>();

  // Everything-is-fine message
  std::string ok_msg = "CompliantActuatorPlugin on " + joint_name + " started with operation mode number [" + std::to_string(operationMode) + "]!";
  ROS_WARN_STREAM(ok_msg);

  // Retrieve joint
  joint = model->GetJoint(joint_name);

  // Compose the topic names
  CompliantActuatorPlugin::topicNames(ns_name, joint_name);

  // Initialize motors, controllers and other parameters
  InitParams(_sdf);


  // Subscribers for the joint commands
  sub_1 = n.subscribe(cmd_ref1_name, 10, &CompliantActuatorPlugin::getRef1_callback, this);
  sub_2 = n.subscribe(cmd_ref2_name, 10, &CompliantActuatorPlugin::getRef2_callback, this);
  if (flag_sub_ext_tau)
  {
    sub_ext = n.subscribe(ext_tau_sub_name, 10, &CompliantActuatorPlugin::getExtTau_callback, this);
  }

  // Publishers for the joint states
  if (flag_pub_el_tau)
  {
    pubEl_1 = n.advertise<std_msgs::Float64>(cmd_pub1_name, 500);
    pubEl_2 = n.advertise<std_msgs::Float64>(cmd_pub2_name, 500);
  }

  if ((operationMode == MotorTorques) || (operationMode == PIDRefs) || (operationMode ==  PIDEqPres))
  {
      std::string motor_state_1_name = ns_name + "/" + joint_name + "/motor_1_state";
      pub_mot_1_state = n.advertise<sensor_msgs::JointState>(motor_state_1_name, 500);

      std::string motor_state_2_name = ns_name + "/" + joint_name + "/motor_2_state";
      pub_mot_2_state = n.advertise<sensor_msgs::JointState>(motor_state_2_name, 500);
  }


  if (flag_pub_state)
  {
    pubL_state = n.advertise<compliant_plugin::state_info>(link_pub_name, 500);
  }

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&CompliantActuatorPlugin::OnUpdate, this, _1));

}


// Compose topic's names according to the type of actuators
void CompliantActuatorPlugin::topicNames(std::string ns_name, std::string joint_name)
{
  // Compose string name for the publishers removing last words from joint name
  cmd_pub1_name = ns_name + "/" + joint_name + "/torque_el1_state";
  cmd_pub2_name = ns_name + "/" + joint_name + "/torque_el2_state";

  // Compose string name for the state publisher
  link_pub_name = ns_name + "/" + joint_name + "/link_state";

  // Compose string for the subscriber of external torque
  ext_tau_sub_name = ns_name + "/" + joint_name + "/external_torque";

  switch(operationMode){
    case(LinkTorque):
        cmd_ref1_name = ns_name + "/" + joint_name + "/torque_command";
        cmd_ref2_name = ns_name + "/" + joint_name + "/disabled";
        break;
    case(LinkPID):
        cmd_ref1_name = ns_name + "/" + joint_name + "/link_command";
        cmd_ref2_name = ns_name + "/" + joint_name + "/disabled";
        break;
    case(SpringRefs):
    case(PIDRefs):
        cmd_ref1_name = ns_name + "/" + joint_name + "/reference_1";
        cmd_ref2_name = ns_name + "/" + joint_name + "/reference_2";
        break;
    case(SpringEqPres):
    case(PIDEqPres):
        cmd_ref1_name = ns_name + "/" + joint_name + "/equilibrium_position";
        cmd_ref2_name = ns_name + "/" + joint_name + "/stiffness_preset";
        break;
    case(MotorTorques):
        cmd_ref1_name = ns_name + "/" + joint_name + "/torque_m1_command";
        cmd_ref2_name = ns_name + "/" + joint_name + "/torque_m2_command";
        break;
  }

}


