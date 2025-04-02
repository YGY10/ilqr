#pragma once
#include <Eigen/Dense>

class PointMassModel {
public:
  PointMassModel(double dt);
  // 运动学模型 x_{k+1} = f(x_k, u_k)
  Eigen::VectorXd update(const Eigen::VectorXd &x,
                         const Eigen::VectorXd &u) const;
  // 计算fx fu
  void linearize(const Eigen::VectorXd &x, const Eigen::VectorXd &u,
                 Eigen::MatrixXd &fx, Eigen::MatrixXd &fu) const;

private:
  double dt_; // 时间步长
};