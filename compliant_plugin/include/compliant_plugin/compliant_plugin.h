#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <compliant_plugin/state_info.h>

#define INITIALIZE_PARAMETER_FROM_TAG( TYPE, VAR, SDF_NAME, NAME, DEF_VALUE ) if ((SDF_NAME)->HasElement(NAME)) {(VAR) = ((SDF_NAME)->GetElement((NAME))->Get<TYPE>()); }else{ (VAR) = (DEF_VALUE); }

namespace gazebo{

  class CompliantActuatorPlugin : public ModelPlugin {

    class DCMotor{

      public:
        DCMotor(const double & _J = 0.0233, //kg m^2
                const double & _D = 0.2698, //N m /(m/s)
                const double & _tauMax = 6.0, // N m
                const double & _maxVel = 6.0, // rad/s
                const double & _minPos = -std::numeric_limits<double>::infinity(),
                const double & _maxPos = std::numeric_limits<double>::infinity(),
                const double & _tauFric = 0.5 // N m  (to check)
                ): J(_J), D(_D), tauMax(_tauMax), maxVel(_maxVel), minPos(_minPos), maxPos(_maxPos), tauFric(_tauFric)
                {
                pos = 0.0;
                vel = 0.0;
                w = 0.0;
                };

        // update function to be called once per step
        void OnUpdate(const double & dT, double & tauMot, const double & tauLoad);

        //state
        double pos; // actual output shaft position
        double vel; // actual output shaft velocity
        double w; // friction equilibrium state

        //parameters
        double maxVel; // max output shaft velocity
        double minPos; // min output shaft position
        double maxPos; // max output shaft position
        double tauFric; // output shaft friction

        double J; // motor shaft inertia
        double D; // motor shaft damping
        double tauMax; // max input torque

        double effort; // total effort to the motor output shaft

      };

    class PIDController{

      public:
        PIDController(const double & _P = 164.0,
                const double & _I = 0.0,
                const double & _D = 41.0
                ): P(_P), I(_I), D(_D)
                {
                  effort = 0.0;
                  posRef = 0.0;
                  velRef = 0.0;
                  errInt = 0.0;
                };

        // update function to be called once per step
        void OnUpdate(const double & dT, const double & pos, const double & vel);

        //state
        double effort; // controller output

        double posRef; // position reference
        double velRef; // velocity reference
        double errInt; // error Integral

        //parameters
        double P; // proportional constant
        double I; // integral constant
        double D; // derivative constant

      };

  public:
    // Constructor
    CompliantActuatorPlugin() : ModelPlugin(){}  //VEDERE!!!!

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);  //TODO

    void OnUpdate(const common::UpdateInfo & info);  //DONE

    void getRef1_callback(const std_msgs::Float64& val_1);  //DONE
    void getRef2_callback(const std_msgs::Float64& val_2);  //DONE
    void getExtTau_callback(const std_msgs::Float64& tauExt);  //DONE
    static double saturate(double val, double max_val);

  protected:

    // spring functions
    virtual void tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out); // change to change actuator
    virtual void eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out); // change to change actuator

    // initialize motors
    virtual void InitParams(sdf::ElementPtr _sdf); // change to change actuator

    // motors
    DCMotor mot_1, mot_2;
    sensor_msgs::JointState mot_1_state, mot_2_state;
    std_msgs::Float64 elastic_tau1, elastic_tau2;

    // controllers
    PIDController ctrl_1, ctrl_2;

    // status
    common::Time oldTime;
    double ref_1;
    double ref_2;
    double ext_tau;
    double T_sample;

    compliant_plugin::state_info joint_info;

    enum OperationModes {LinkTorque = 0 , LinkPID = 1, SpringRefs = 2 , SpringEqPres = 3, MotorTorques = 4, PIDRefs = 5, PIDEqPres = 6};
    OperationModes operationMode;

    void OnUpdateLinkTorque(const common::UpdateInfo & info);
    void OnUpdateLinkPID(const common::UpdateInfo & info);
    void OnUpdateSpringRefs(const common::UpdateInfo & info);
    void OnUpdateSpringEqPres(const common::UpdateInfo & info);
    void OnUpdateMotorTorques(const common::UpdateInfo & info);
    void OnUpdatePIDRefs(const common::UpdateInfo & info);
    void OnUpdatePIDEqPres(const common::UpdateInfo & info);

    // helper functions
    void Publish(const double & tauEl_1, const double & tauEl_2,
                                      const double & tauEl_L, const double & sigmaEl_L,
                                      const double & q, const double & dq,
                                      const double & r_1, const double & r_2 );

    void PublishMotors(double m1_pos, double m1_vel, double m1_eff, double m2_pos, double m2_vel, double m2_eff);
    void topicNames(std::string ns_name, std::string joint_name);

    // ROS handle
    ros::NodeHandle n;

    // publishers
    ros::Publisher pubEl_1;   // for elastic torque to motor 1
    ros::Publisher pubEl_2;   // for elastic torque to motor 2
    ros::Publisher pub_mot_1_state;   // for motor 1 state
    ros::Publisher pub_mot_2_state;   // for motor 2 state
    ros::Publisher pubL_state;// for joint state


    // subscribers
    ros::Subscriber sub_1;  // for references 1
    ros::Subscriber sub_2;  // for references 2
    ros::Subscriber sub_ext;// for eternal torque

    // Pointer to the model
    physics::ModelPtr model;
    physics::LinkPtr link;

    // Pointer to output shaft joint
    physics::JointPtr joint;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Command name to create a specific publisher and subscribers
    std::string cmd_pub1_name;
    std::string cmd_pub2_name;
    std::string cmd_ref1_name;
    std::string cmd_ref2_name;

    // String names of the topic from which retrieve the desired motor variables
    std::string link_pub_name;

    // String name for the external torque
    std::string ext_tau_sub_name;

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name = "toBEassigned";

    // Enable publish and subscribe to specific topics (default "false")
    bool flag_pub_el_tau = false;
    bool flag_pub_state = false;
    bool flag_sub_ext_tau = false;

  };

}
