#include <viactors_plugins/awas_ii_plugin.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Elastic torques model
void AwASActuatorPlugin::tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out){

    double defl = saturate(qL_in - q1_in, max_def);

    // elastic torque
    t1_out = -2*Ks*pow(((L/r_t)*(q2_in/(L-q2_in))),2) * sin(defl)*cos(defl);
    tL_out = t1_out;

    // link stiffness
    sigmaL_out = 2*Ks*pow(((L/r_t)*(q2_in/(L-q2_in))),2) * (2*pow(cos(defl),2) - 1);

};  

// Function to get equilibirum position and preset
void AwASActuatorPlugin::eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out){

    ql_out = eq_in;
    q2_out = pres_in;

};  

// Initialize all parameters
void AwASActuatorPlugin::InitParams(sdf::ElementPtr _sdf){

    // Setup default parameter values
    //springs parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, Ks, _sdf, "spring_Ks", 10 ) // 1 /rad
    INITIALIZE_PARAMETER_FROM_TAG( double, l_0, _sdf, "spring_l_0", 1 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, L, _sdf, "spring_L", 0.05 ) // 1 /rad
    INITIALIZE_PARAMETER_FROM_TAG( double, r_t, _sdf, "spring_r_t", 0.015 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, max_def, _sdf, "max_def", 0.3 ) // rad

    // motors parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.J, _sdf, "mot_J", 0.0233 ) //kg m^2
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.D, _sdf, "mot_D", 0.0019 ) //N m /(m/s)
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauMax, _sdf, "mot_tauMax", 6.0 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxVel, _sdf, "mot_maxVel", 6.0 )   // rad/s
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.minPos, _sdf, "mot_minPos", -std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxPos, _sdf, "mot_maxPos", std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauFric, _sdf, "mot_tauFric", 0.5 ) // N m  (to check)

    //controllers parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_P", 125.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_I", 0.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_D", 4.5)

    // Change control
    if (operationMode == LinkPID){
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_P", 0.8 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_I", 0.0 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_D", 0.2 )
    }

};


