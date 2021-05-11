#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <eigen3/Eigen/Eigen>
#include <plugin_manager/QbAdvanceRefEq.h>
#include <std_msgs/Float64MultiArray.h> 
#include <sensor_msgs/JointState.h>
#include <plugin_manager/state_info.h>

# define PI 3.14159

int joints_n;
double T_sample;
std::string ns_name;

std::vector<ros::Publisher> pub_ref1, pub_ref2;
ros::Publisher pub_robot_state, pub_motor1_state, pub_motor2_state;

std::vector<ros::Subscriber> sub_joints_state;
std::vector<ros::Subscriber> sub_mot1_state;
std::vector<ros::Subscriber> sub_mot2_state;
ros::Subscriber sub_ref1, sub_ref2;

std::vector<double> q, dq, tau, stiff;
std::vector<double> th_1, th_2, dth_1, dth_2, tau_1, tau_2;

std_msgs::Float64 msg_ref1, msg_ref2;
std_msgs::Float64MultiArray ref_1, ref_2;

std::vector<std::string> joint_name, ref1_topic_name, ref2_topic_name;
std::vector<std::string> topics_ref1, topics_ref2, topics_state;
std::vector<std::string> mot1_state, mot2_state;
std::string topic_state_name;
std::string  topic_mot1state_name, topic_mot2state_name;

sensor_msgs::JointState robot_state, state_1, state_2;

void JointStateCallback(const plugin_manager::state_infoConstPtr &msg, int i)
{
  q[i] = msg->q;
  dq[i] = msg->dq;
  tau[i] = msg->tau;
  stiff[i] = msg->stiff;
}

// Callbacks for the references
void Ref1_Callback(const std_msgs::Float64MultiArray &msg)
{
  ref_1 = msg;
}
void Ref2_Callback(const std_msgs::Float64MultiArray &msg)
{
  ref_2 = msg;
}
// Callbacks for the motors data
void Mot1StateCallback(const sensor_msgs::JointStateConstPtr &msg, int i)
{
/*  msg.position.resize(1);
  msg->velocity.resize(1);
  msg->effort.resize(1);*/
  th_1[i] = msg->position[0];
  dth_1[i] = msg->velocity[0];
  tau_1[i] = msg->effort[0];
}
void Mot2StateCallback(const sensor_msgs::JointStateConstPtr &msg, int i)
{
/*  msg->position.resize(1);
  msg->velocity.resize(1);
  msg->effort.resize(1);*/
  th_2[i] = msg->position[0];
  dth_2[i] = msg->velocity[0];
  tau_2[i] = msg->effort[0];
}

void Initialization(ros::NodeHandle n_)
{
  // Check if namespace is passed and get it
  if (!n_.getParam("namespace", ns_name))
  {
    ROS_ERROR("Specify namespace of robot");
    exit(1);
  }
  // Check if joint number is passed and get it
  if (!n_.getParam("joints_number", joints_n))
  {
    ROS_ERROR("Specify number of joint");
    exit(1);
  }
  // Check if sample time is passed and get it
  if (!n_.getParam("T_sample", T_sample))
  {
    ROS_ERROR("Specify the sample time");
    exit(1);
  }

  // Resize array variables
  pub_ref1.resize(joints_n);
  pub_ref2.resize(joints_n);
  sub_joints_state.resize(joints_n);
  sub_mot1_state.resize(joints_n);
  sub_mot2_state.resize(joints_n);
  joint_name.resize(joints_n);
  ref1_topic_name.resize(joints_n);
  ref2_topic_name.resize(joints_n);
  topics_ref1.resize(joints_n);
  topics_ref2.resize(joints_n);
  topics_state.resize(joints_n);
  mot1_state.resize(joints_n);
  mot2_state.resize(joints_n);
  ref_1.data.resize(joints_n);
  ref_2.data.resize(joints_n);
  q.resize(joints_n);
  dq.resize(joints_n);
  tau.resize(joints_n);
  stiff.resize(joints_n);
  th_1.resize(joints_n);
  dth_1.resize(joints_n);
  tau_1.resize(joints_n);
  th_2.resize(joints_n);
  dth_2.resize(joints_n);
  tau_2.resize(joints_n);
  robot_state.name.resize(joints_n);
  robot_state.position.resize(joints_n);
  robot_state.velocity.resize(joints_n);
  robot_state.effort.resize(joints_n);
  state_1.name.resize(joints_n);
  state_1.position.resize(joints_n);
  state_1.velocity.resize(joints_n);
  state_1.effort.resize(joints_n);
  state_2.name.resize(joints_n);
  state_2.position.resize(joints_n);
  state_2.velocity.resize(joints_n);
  state_2.effort.resize(joints_n);

  // Check if joint name vector is passed and get it
  if (!n_.getParam("joints_name", joint_name))
  {
    ROS_ERROR("Specify the joint name vector");
    exit(1);
  }
  // Subscribe to equilibirum position topics
  if (!n_.getParam("ref1_topic_name", ref1_topic_name) || !n_.getParam("ref2_topic_name", ref2_topic_name))
  {
    ROS_ERROR("Specify the name of the subscriber topic");
    exit(1);
  }

  // Create the subscriber topics' names
  for(int i = 0; i<joints_n; i++)
  {
    topics_ref1[i]  = "/" +  ns_name + "/" + joint_name[i] + ref1_topic_name[i];
    topics_ref2[i]  = "/" +  ns_name + "/" + joint_name[i] + ref2_topic_name[i];
    topics_state[i] = "/" +  ns_name + "/" + joint_name[i] + "/link_state";
    mot1_state[i] = "/" +  ns_name + "/" + joint_name[i] + "/motor_1_state";
    mot2_state[i] = "/" +  ns_name + "/" + joint_name[i] + "/motor_2_state";
  }
  topic_state_name = "/" +  ns_name + "/robot_state";
  topic_mot1state_name = "/" +  ns_name + "/motor_1_state";
  topic_mot2state_name = "/" +  ns_name + "/motor_2_state";

  // Advertise topics' publishers
  for(int i = 0; i<joints_n; i++)
  {
    pub_ref1[i] =  n_.advertise<std_msgs::Float64>(topics_ref1[i], 10);
    pub_ref2[i] =  n_.advertise<std_msgs::Float64>(topics_ref2[i], 10);
  }
  // Advertise robot state and motors state publishers
  pub_robot_state = n_.advertise<sensor_msgs::JointState>(topic_state_name, 10);
  pub_motor1_state = n_.advertise<sensor_msgs::JointState>(topic_mot1state_name, 10);
  pub_motor2_state = n_.advertise<sensor_msgs::JointState>(topic_mot2state_name, 10);

  // Subscribe to the joint state
  for(int i = 0; i<joints_n; i++)
  {
    sub_joints_state[i] = n_.subscribe<plugin_manager::state_info>(topics_state[i], 10, boost::bind(JointStateCallback, _1, i));
    sub_mot1_state[i] = n_.subscribe<sensor_msgs::JointState>(mot1_state[i], 10, boost::bind(Mot1StateCallback, _1, i));
    sub_mot2_state[i] = n_.subscribe<sensor_msgs::JointState>(mot2_state[i], 10, boost::bind(Mot2StateCallback, _1, i));
  }

  std::string sub_ref1_name = ns_name + "/reference_1";
  std::string sub_ref2_name = ns_name + "/reference_2";

  sub_ref1 = n_.subscribe(sub_ref1_name, 10, &Ref1_Callback);
  sub_ref2 = n_.subscribe(sub_ref2_name, 10, &Ref2_Callback);

}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "plugin_manager_node");
  
  ros::NodeHandle n_;
  
  Initialization(n_);
 
  ros::Rate r(1/T_sample);

  while(ros::ok())
  {

    for(int i = 0; i<joints_n; i++)
    {
      msg_ref1.data = ref_1.data[i];
      msg_ref2.data = ref_2.data[i];

      pub_ref1[i].publish(msg_ref1);
      pub_ref2[i].publish(msg_ref2);

      // assign stamp time
      ros::Time act_time = ros::Time::now();
      state_1.header.stamp = act_time;
      state_2.header.stamp = act_time;
      robot_state.header.stamp = act_time;

      // Fill data for the robot's link
      robot_state.name[i] = joint_name[i];
      robot_state.position[i] = q[i];
      robot_state.velocity[i] = dq[i];
      robot_state.effort[i] = tau[i];

      // Fill data for the motor 1 variables of the robot
      state_1.name[i] = joint_name[i];
      state_1.position[i] = th_1[i];
      state_1.velocity[i] = dth_1[i];
      state_1.effort[i] = tau_1[i];

      // Fill data for the motor 2 variables of the robot
      state_2.name[i] = joint_name[i];
      state_2.position[i] = th_2[i];
      state_2.velocity[i] = dth_2[i];
      state_2.effort[i] = tau_2[i];
    }

    pub_robot_state.publish(robot_state);
    pub_motor1_state.publish(state_1);
    pub_motor2_state.publish(state_2);

    ros::spinOnce();
    r.sleep();
	  
  }

  return 0;
}