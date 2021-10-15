#include <viactors_plugins/sea_plugin.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Elastic torques model
void SEAPlugin::tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out){
	
    // Elastic model parameters
    double defl = saturate(qL_in - q1_in, max_def);

    // elastic torque
    t1_out = -k*defl;
    tL_out = t1_out;

    // link stiffness
    sigmaL_out = k;

}; 

// Function to get equilibirum position and preset
void SEAPlugin::eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out){

	ql_out = eq_in;

}; 

// Initialize all parameters
void SEAPlugin::InitParams(sdf::ElementPtr _sdf){

    // Show the actuator type for info
    ROS_WARN_STREAM("Actuator type: SEA");

    // Setup default parameter values
    //springs parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, k, _sdf, "spring_k", 10 ) // N m /rad
    INITIALIZE_PARAMETER_FROM_TAG( double, max_def, _sdf, "max_def", 0.8 ) // rad

    // motors parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.J, _sdf, "mot_J", 0.0233 ) //kg m^2
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.D, _sdf, "mot_D", 0.2698 ) //N m /(m/s)
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauMax, _sdf, "mot_tauMax", 6.0 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxVel, _sdf, "mot_maxVel", 6.0 )   // rad/s
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.minPos, _sdf, "mot_minPos", -std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxPos, _sdf, "mot_maxPos", std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauFric, _sdf, "mot_tauFric", 0.0 ) // N m  (to check)

    //controllers parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_P", 125.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_I", 0.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_D", 4.5 )

    // Change control
    if (operationMode == LinkPID){
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_P", 0.8 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_I", 0.0 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_D", 0.2 )
    }	
    
}; 


