#include "models/point_mass_model.hpp"
#include <iostream>

int main() {
  PointMassModel model(0.1); // dt = 0.1s

  Eigen::VectorXd x(2);
  x << 0.0, 1.0; // 初始位置 0，速度 1 m/s

  Eigen::VectorXd u(1);
  u << 2.0; // 加速度 2 m/s^2

  Eigen::VectorXd x_next = model.update(x, u);
  std::cout << "Next state: \n" << x_next << std::endl;

  return 0;
}
