#include <viactors_plugins/bavs_plugin.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Elastic torques model
void BAVSPlugin::tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out){
    
    double defl1 = saturate(q1_in - qL_in, max_def);
    double defl2 = saturate(q2_in - qL_in, max_def);

    // elastic torques
    t1_out = -k*( exp(-k_e*defl1) * (a - b*exp(2*k_e*defl1)) );
    t2_out = -k*( exp(-k_e*defl2) * (b - a*exp(2*k_e*defl2)) );
    tL_out = t1_out + t2_out;

    // link stiffness
    double sig1 = -k_e*t1_out + 2*k_e*k*( b*exp(k_e*defl1) );
    double sig2 = -k_e*t2_out + 2*k_e*k*( a*exp(k_e*defl2) );
    sigmaL_out = sig1 + sig2;

};

// Function to get equilibirum position and preset
void BAVSPlugin::eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out){
	
    ql_out = eq_in + pres_in;
    q2_out = eq_in - pres_in;

};

// Initialize all parameters
void BAVSPlugin::InitParams(sdf::ElementPtr _sdf){

    // Show the actuator type for info
    ROS_WARN_STREAM("Actuator type: DLR's BAVS");

    // Setup default parameter values
    //springs parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, a, _sdf, "spring_a", 1.21*pow(10,-6) ) // 
    INITIALIZE_PARAMETER_FROM_TAG( double, b, _sdf, "spring_b", 2.38*pow(10,-6) ) // 
    INITIALIZE_PARAMETER_FROM_TAG( double, k, _sdf, "spring_k", 1.60*pow(10,5) ) // 
    INITIALIZE_PARAMETER_FROM_TAG( double, k_e, _sdf, "spring_k_e", 8.71 ) // 
    INITIALIZE_PARAMETER_FROM_TAG( double, max_def, _sdf, "max_def", 0.27 ) // rad

    // motors parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.J, _sdf, "mot_1_J", 0.0233 ) //kg m^2
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.D, _sdf, "mot_1_D", 0.2698 ) //N m /(m/s)
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauMax, _sdf, "mot_1_tauMax", 6.0 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxVel, _sdf, "mot_1_maxVel", 6.0 )   // rad/s
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.minPos, _sdf, "mot_1_minPos", -std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxPos, _sdf, "mot_1_maxPos", std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauFric, _sdf, "mot_1_tauFric", 0.0 ) // N m  (to check)

    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.J, _sdf, "mot_2_J", 0.0233 ) //kg m^2
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.D, _sdf, "mot_2_D", 0.2698 ) //N m /(m/s)
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.tauMax, _sdf, "mot_2_tauMax", 6.0 )  // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.maxVel, _sdf, "mot_2_maxVel", 6.0 ) // rad/s
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.minPos, _sdf, "mot_2_minPos", -std::numeric_limits<double>::infinity() )  // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.maxPos, _sdf, "mot_2_maxPos", std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.tauFric, _sdf, "mot_2_tauFric", 0.0 )

    //controllers parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_1_P", 125.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_1_I", 0.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_1_D", 4.5 )

    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_2.P, _sdf, "ctrl_2_P", 125.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_2.I, _sdf, "ctrl_2_I", 0.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_2.D, _sdf, "ctrl_2_D", 4.5 )

    // Change control
    if (operationMode == LinkPID){
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_1_P", 0.8 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_1_I", 0.0 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_1_D", 0.2 )
    }

}; 
