#include <viactors_plugins/qbmove_plugin.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Elastic torques model
void qbMovePlugin::tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out){

	// Elastic model parameters
    double defl = saturate(qL_in - q1_in, max_def);
    double def2 = saturate(qL_in - q2_in, max_def);

    // elastic torques
    t1_out = - k1*sinh(a1*defl);
    t2_out = - k2*sinh(a2*def2);
 
    tL_out = t1_out + t2_out;

    // link stiffness
    sigmaL_out = a1*k1*cosh(a1*defl) + a2*k2*cosh(a2*def2);

}; 

// Function to get equilibirum position and preset
void qbMovePlugin::eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out){

    ql_out = eq_in + pres_in;
    q2_out = eq_in - pres_in;

}; 

// Initialize all parameters
void qbMovePlugin::InitParams(sdf::ElementPtr _sdf){

    // Setup default parameter values
    //springs parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, a1, _sdf, "spring_a1", 8.9992 ) // 1 /rad
    INITIALIZE_PARAMETER_FROM_TAG( double, k1, _sdf, "spring_k1", 0.0019 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, a2, _sdf, "spring_a2", 8.9992 ) // 1 /rad
    INITIALIZE_PARAMETER_FROM_TAG( double, k2, _sdf, "spring_k2", 0.0019 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, max_def, _sdf, "max_def", 0.8 ) // rad

    // motors parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.J, _sdf, "mot_1_J", 0.0233 ) //kg m^2
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.D, _sdf, "mot_1_D", 0.0019 ) //N m /(m/s)
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.tauMax, _sdf, "mot_1_tauMax", 6.0 ) // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxVel, _sdf, "mot_1_maxVel", 0.0019 )   // rad/s
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.minPos, _sdf, "mot_1_minPos", -std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.maxPos, _sdf, "mot_1_maxPos", std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_1.J, _sdf, "mot_1_tauFric", 0.5 ) // N m  (to check)

    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.J, _sdf, "mot_2_J", 0.0233 ) //kg m^2
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.D, _sdf, "mot_2_D", 0.0019 ) //N m /(m/s)
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.tauMax, _sdf, "mot_2_tauMax", 6.0 )  // N m
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.maxVel, _sdf, "mot_2_maxVel", 0.0019 ) // rad/s
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.minPos, _sdf, "mot_2_minPos", -std::numeric_limits<double>::infinity() )  // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.maxPos, _sdf, "mot_2_maxPos", std::numeric_limits<double>::infinity() ) // rad
    INITIALIZE_PARAMETER_FROM_TAG( double, mot_2.J, _sdf, "mot_2_tauFric", 0.5 )

    //controllers parameters
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_1_P", 250.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_1_I", 0.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_1_D", 40.0 )

    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_2.P, _sdf, "ctrl_2_P", 250.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_2.I, _sdf, "ctrl_2_I", 0.0 )
    INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_2.D, _sdf, "ctrl_2_D", 40.0 )

    // Change control
    if (operationMode == LinkPID){
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.P, _sdf, "ctrl_1_P", 0.8 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.I, _sdf, "ctrl_1_I", 0.0 )
        INITIALIZE_PARAMETER_FROM_TAG( double, ctrl_1.D, _sdf, "ctrl_1_D", 0.2 )
    }

} 