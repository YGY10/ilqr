#pragma once
#include "point_mass_model.hpp"
#include <Eigen/Dense>
#include <vector>

class ILQR {
public:
  using DynamicsFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &,
                                                     const Eigen::VectorXd &)>;
  using LinearizeFunc =
      std::function<void(const Eigen::VectorXd &, const Eigen::VectorXd &,
                         Eigen::MatrixXd &, Eigen::MatrixXd &)>;
  ILQR(DynamicsFunc dynamics, LinearizeFunc linearize,
       const Eigen::VectorXd &x0, const Eigen::VectorXd &x_goal, int horizon,
       double dt);

  // 前向传播函数
  void backwardPass(std::vector<Eigen::MatrixXd> &K,
                    std::vector<Eigen::VectorXd> &k);

  void forwardPass(const std::vector<Eigen::MatrixXd> &K,
                   const std::vector<Eigen::VectorXd> &k, double alpha = 1.0);

  // 求解函数
  void solve(int max_iterations = 100);

  // 获取优化后的状态轨迹/控制序列
  const std::vector<Eigen::VectorXd> &getStateTrajectory() const;
  const std::vector<Eigen::VectorXd> &getControlTrajectory() const;

private:
  DynamicsFunc dynamics_;
  LinearizeFunc linearize_;
  double dt_;
  int horizon_;
  Eigen::VectorXd x0_;
  Eigen::VectorXd x_goal_;

  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
};