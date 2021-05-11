#include <compliant_plugin/compliant_plugin.h>

namespace gazebo{

  class TEMPLATEPlugin : public CompliantActuatorPlugin {

  public:
    TEMPLATEPlugin() : CompliantActuatorPlugin(){}

    void tauElastic(const double & q1_in, const double & q2_in, const double & qL_in, double & t1_out, double & t2_out, double & tL_out, double & sigmaL_out); 
    void eqPres2Refs(const double & eq_in, const double & pres_in, double & ql_out, double & q2_out);  
    void InitParams(sdf::ElementPtr _sdf);

  private:
    // DEFINE HERE YOUR MODEL PARAMETERS

  };
  
  GZ_REGISTER_MODEL_PLUGIN(TEMPLATEPlugin)

}
