#include "ilqr.hpp"
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

  std::vector<Eigen::MatrixXd> K;
  std::vector<Eigen::VectorXd> k;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // forward rollout (using current control sequence)
    for (int i = 0; i < horizon_; ++i) {
      xs_[i + 1] = dynamics_(xs_[i], us_[i]);
    }

    // backward pass
    backwardPass(K, k);

    // forward pass (with line search if needed)
    forwardPass(K, k, 1.0);

    std::cout << "Iteration " << iter << " complete\n";
  }
}

void ILQR::backwardPass(std::vector<Eigen::MatrixXd> &K,
                        std::vector<Eigen::VectorXd> &k) {
  int n = x0_.size(); // 状态维度
  int m = 1;          // 控制维度

  // 代价参数
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n) * 100.0;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m) * 10.0;

  Eigen::MatrixXd Qf = Q;

  // 初始化终端代价
  Eigen::MatrixXd P = Qf;
  Eigen::VectorXd p = -Qf * x_goal_;

  // resize 输出
  K.resize(horizon_);
  k.resize(horizon_);

  for (int i = horizon_ - 1; i >= 0; --i) {
    // 当前状态和控制
    const Eigen::VectorXd &x = xs_[i];
    const Eigen::VectorXd &u = us_[i];

    // 与目标差值
    Eigen::VectorXd dx = x - x_goal_;

    // 一阶导
    Eigen::VectorXd lx = 2 * Q * dx;
    Eigen::VectorXd lu = 2 * R * u;

    // 二阶导
    Eigen::MatrixXd lxx = 2 * Q;
    Eigen::MatrixXd luu = 2 * R;
    Eigen::MatrixXd lux = Eigen::MatrixXd::Zero(m, n);

    // 线性化系统
    Eigen::MatrixXd A, B;
    linearize_(x, u, A, B);

    // Q 函数展开
    Eigen::VectorXd Qx = lx + A.transpose() * p;
    Eigen::VectorXd Qu = lu + B.transpose() * p;
    Eigen::MatrixXd Qxx = lxx + A.transpose() * P * A;
    Eigen::MatrixXd Quu = luu + B.transpose() * P * B;
    Eigen::MatrixXd Qux = lux + B.transpose() * P * A;

    // 反馈律
    Eigen::MatrixXd Quu_inv = Quu.inverse();
    K[i] = -Quu_inv * Qux;
    k[i] = -Quu_inv * Qu;

    // 更新值函数
    P = Qxx + K[i].transpose() * Quu * K[i] + K[i].transpose() * Qux +
        Qux.transpose() * K[i];
    p = Qx + K[i].transpose() * Quu * k[i] + K[i].transpose() * Qu +
        Qux.transpose() * k[i];
  }
}

void ILQR::forwardPass(const std::vector<Eigen::MatrixXd> &K,
                       const std::vector<Eigen::VectorXd> &k, double alpha) {
  xs_[0] = x0_; // 初始状态

  for (int i = 0; i < horizon_; ++i) {
    // 当前状态误差
    Eigen::VectorXd dx = xs_[i] - xs_[i]; // 初始轨迹，dx = 0

    // 控制更新
    Eigen::VectorXd du = alpha * k[i] + K[i] * dx;

    us_[i] += du;
    us_[i] = us_[i].cwiseMax(-1.0).cwiseMin(1.0); // 加速度限制

    // 模拟下一个状态
    xs_[i + 1] = dynamics_(xs_[i], us_[i]);
  }
}

const std::vector<Eigen::VectorXd> &ILQR::getStateTrajectory() const {
  return xs_;
}

const std::vector<Eigen::VectorXd> &ILQR::getControlTrajectory() const {
  return us_;
}