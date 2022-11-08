//
// Created by zhanghairong on 22-9-20.
//
#include <apriltag_localization/pose_system.h>
#include <Eigen/Dense>
#include "matplotlibcpp.h"
int main(){
  namespace plt = matplotlibcpp;
  apriltag_localization::PoseSystem system;
  system.dt = 0.1;
  Eigen::VectorXf state(6);
  state.setZero();
  state(3) = 0;
  Eigen::VectorXf control(12);
  control.setZero();
  control(0) = 0.1; // v0x
  control(1) = 0.1; // v0y
  control(5) = 0.01; // w0 in z
  control(6) = 0.1; // v1
  control(7) = 0.1; // v1
  control(11) = 0.01; // w1 in z
//  control(0) = 0.1; // v0
//
//  control(6) = 0.1; // v1

  std::vector<double> state_x, state_y;
  for (int i = 0; i < 1000; ++i)
  {
    state = system.f(state, control);
    control(5) = control(11);
    control(11) += 0.01;
    state_x.push_back(state(3));
    state_y.push_back(state(4));

    std::cout << state.transpose() << std::endl;
  }

  plt::figure_size(840,840);
  plt::named_plot("origin", state_x, state_y, "r");
  plt::title("rk4");
  plt::grid(true);
  plt::show();

  return 0;
}