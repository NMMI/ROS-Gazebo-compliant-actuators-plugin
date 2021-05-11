#include <viactors_plugins/template_plugin.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Elastic torques model
void TEMPLATEPlugin::tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out){
	
    // YOUR ELASTIC MODEL GOES HERE

}; 

// Function to get equilibirum position and preset
void TEMPLATEPlugin::eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out){

	// YOUR MODEL GOES HERE

}; 

// Initialize all parameters
void TEMPLATEPlugin::InitParams(sdf::ElementPtr _sdf){

    // YOUR INITIALIZATION GOES HERE	
    
}; 


