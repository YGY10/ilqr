#include "ilqr.hpp"
#include "models/point_mass_model.hpp"
#include "plot/matplotlibcpp.h" // 画图
#include <iostream>

namespace plt = matplotlibcpp;

int main() {
  double dt = 0.1;
  int horizon = 50;

  PointMassModel model(dt);

  Eigen::VectorXd x0(2);
  x0 << 0.0, 0.0;

  Eigen::VectorXd x_goal(2);
  x_goal << 10.0, 5.0;

  auto dynamics = [&model](const Eigen::VectorXd &x,
                           const Eigen::VectorXd &u) -> Eigen::VectorXd {
    return model.update(x, u);
  };

  auto linearize = [&model](const Eigen::VectorXd &x, const Eigen::VectorXd &u,
                            Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
    model.linearize(x, u, A, B);
  };

  ILQR ilqr(dynamics, linearize, x0, x_goal, horizon, dt);
  ilqr.solve(20);

  const auto &xs = ilqr.getStateTrajectory();
  const auto &us = ilqr.getControlTrajectory();

  std::cout << "\n=== Optimized Trajectory ===\n";
  for (int i = 0; i < xs.size(); ++i) {
    std::cout << "Step " << i << ": x = " << xs[i].transpose();
    if (i < us.size()) {
      std::cout << ", u = " << us[i].transpose();
    }
    std::cout << "\n";
  }

  // ✅ 可视化
  std::vector<double> position, velocity, acceleration, time;

  for (int i = 0; i < xs.size(); ++i) {
    position.push_back(xs.at(i)(0));                           // 位置 x
    velocity.push_back(xs.at(i)(1));                           // 速度 v
    acceleration.push_back(i < us.size() ? us.at(i)(0) : 0.0); // 加速度 a
    time.push_back(i * dt);                                    // 时间轴
  }

  plt::figure(1);
  plt::plot(time, position);
  plt::title("Position");
  plt::xlabel("Time [s]");
  plt::ylabel("Position [m]");
  plt::grid(true);

  plt::figure(2);
  plt::plot(time, velocity);
  plt::title("Velocity");
  plt::xlabel("Time [s]");
  plt::ylabel("Velocity [m/s]");
  plt::grid(true);

  plt::figure(3);
  plt::plot(time, acceleration);
  plt::title("Acceleration");
  plt::xlabel("Time [s]");
  plt::ylabel("Acceleration [m/s^2]");
  plt::grid(true);

  // 二选一：显示 or 保存
  plt::show();
  // plt::save("trajectory.png"); // 无 GUI 时用这个

  return 0;
}
