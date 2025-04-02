#include "point_mass_model.hpp"

PointMassModel::PointMassModel(double dt) : dt_(dt){};

Eigen::VectorXd PointMassModel::update(const Eigen::VectorXd &x,
                                       const Eigen::VectorXd &u) const {
  Eigen::VectorXd x_next(2);
  double p = x(0);
  double v = x(1);
  double a = u(0);

  x_next(0) = p + v * dt_;
  x_next(1) = v + a * dt_;
  return x_next;
}

void PointMassModel::linearize(const Eigen::VectorXd &x,
                               const Eigen::VectorXd &u, Eigen::MatrixXd &fx,
                               Eigen::MatrixXd &fu) const {
  // fx: ∂f/∂x
  fx = Eigen::MatrixXd::Identity(2, 2);
  fx(0, 1) = dt_; // ∂p_next / ∂v

  // fu: ∂f/∂u
  fu = Eigen::MatrixXd::Zero(2, 1);
  fu(1, 0) = dt_; // ∂v_next / ∂a
}