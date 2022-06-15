#include "rbdl/rbdl.h"
#include "rbdl/rbdl_utils.h"
#include "luamodel.h"

#include <iostream>
#include <iomanip>
#include <sstream>
using namespace std;
using namespace RigidBodyDynamics::Math;

int main (int argc, char *argv[]) {
  RigidBodyDynamics::Model model;
  bool result;
  
  result = RigidBodyDynamics::Addons::LuaModelReadFromFile(
							   "sampleconstrainedmodel.lua", &model, true);


  cout << "Reading result " << result << endl;
  VectorNd q_zero (VectorNd::Zero (model.q_size));
  VectorNd qdot_zero (VectorNd::Zero (model.qdot_size));
  RigidBodyDynamics::UpdateKinematics (model, q_zero, qdot_zero, qdot_zero);
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      if (model.mBodies[i].mIsVirtual)
        continue;

      SpatialRigidBodyInertia rbi_base = model.X_base[i].apply(model.I[i]);
      Vector3d body_com = rbi_base.h / rbi_base.m;
      cout << setw(12) <<model.GetBodyName (i) << ": " << setw(10) <<  casadi::MX::evalf(body_com.transpose()) << endl;    }

}
