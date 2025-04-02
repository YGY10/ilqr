#include "include/ilqr.hpp"
#include <iostream>

ILQR::ILQR(DynamicsFunc dynamics, LinearizeFunc linearize,
           const Eigen::VectorXd &x0, const Eigen::VectorXd &x_goal,
           int horizon, double dt)
    : dynamics_(dynamics), linearize_(linearize), x0_(x0), x_goal_(x_goal),
      horizon_(horizon), dt_(dt) {

  int state_dim = x0.size();
  int control_dim = 1;

  xs_.resize(horizon_ + 1, Eigen::VectorXd::Zero(state_dim));
  us_.resize(horizon_, Eigen::VectorXd::Zero(control_dim));

  xs_[0] = x0_;
}

void ILQR::solve(int max_iterations) {
  std::cout << "Starting iLQR solve...\n";
  for (int iter = 0; iter < max_iterations; ++iter) {
    // TODO: 实现 backwardPass 和 forwardPass
    std::cout << "Iteration " << iter << " complete\n";
  }
}

const std::vector<Eigen::VectorXd> &ILQR::getStateTrajectory() const {
  return xs_;
}

const std::vector<Eigen::VectorXd> &ILQR::getControlTrajectory() const {
  return us_;
}