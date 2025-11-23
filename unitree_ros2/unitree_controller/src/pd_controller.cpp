#include "unitree_controller/pd_controller.hpp"

namespace unitree_controller
{

PDController::PDController(const Vector12d& q_j, const Vector12d& dqJ, const Vector12d& tau_j, 
                           const Vector12d& k_p, const Vector12d& k_d)
  : qJ_cmd_(q_j),  
    dqJ_cmd_(dqJ),  
    tauJ_cmd_(tau_j),  
    Kp_cmd_(k_p),  
    Kd_cmd_(k_d) {
  if (k_p.minCoeff() < 0.0) {
    throw std::invalid_argument("[PDController] k_p.minCoeff() must be non-negative");
  }
  if (k_d.minCoeff() < 0.0) {
    throw std::invalid_argument("[PDController] k_p.minCoeff() must be non-negative");
  }
}


PDController PDController::ZeroTorqueController() 
{
  const Vector12d q_j = Vector12d::Constant(UNITREE_LEGGED_SDK::PosStopF);
  const Vector12d dqJ = Vector12d::Constant(UNITREE_LEGGED_SDK::VelStopF);
  const Vector12d tau_j = Vector12d::Zero();
  const Vector12d k_p = Vector12d::Zero();
  const Vector12d k_d = Vector12d::Zero();
  return PDController(std::move(q_j), std::move(dqJ), std::move(tau_j), std::move(k_p), std::move(k_d));
}


PDController PDController::StandingUpController() 
{
  Vector12d q_j;
  q_j << 0.0, 0.67, -1.3, 
        0.0, 0.67, -1.3, 
        0.0, 0.67, -1.3, 
        0.0, 0.67, -1.3;
  const Vector12d dqJ = Vector12d::Zero();
  const Vector12d tau_j = Vector12d::Zero();
  const Vector12d k_p = Vector12d::Constant(20.0);
  const Vector12d k_d = Vector12d::Constant(10.0);
  return PDController(std::move(q_j), std::move(dqJ), std::move(tau_j), std::move(k_p), std::move(k_d));
}


PDController PDController::SittingDownController() 
{
  Vector12d q_j;
  q_j << 0, 1.0, -2.5, 
        0, 1.0, -2.5, 
        0, 1.0, -2.5, 
        0, 1.0, -2.5;
  const Vector12d dqJ = Vector12d::Zero();
  const Vector12d tau_j = Vector12d::Zero();
  const Vector12d k_p = Vector12d::Constant(10.0);
  const Vector12d k_d = Vector12d::Constant(15.0);
  return PDController(std::move(q_j), std::move(dqJ), std::move(tau_j), std::move(k_p), std::move(k_d));
}

} // namespace unitree_controller