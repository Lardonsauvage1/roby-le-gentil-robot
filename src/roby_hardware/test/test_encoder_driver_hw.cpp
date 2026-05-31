// Test hardware (binaire executable) du driver encodeur C++.
//
// Equivalent du POC Python `encoder_publisher.py` (juste la partie lecture +
// conversion, sans publication ROS) — sert a valider que le port C++ donne
// les memes angles joint que la version Python avant integration dans
// RobySystem.
//
// A executer sur Pi5 :
//     source ~/ros2_ws/install/setup.bash
//     ~/ros2_ws/install/roby_hardware/lib/roby_hardware/test_encoder_driver_hw

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "roby_hardware/encoder_driver.hpp"

int main()
{
  using roby_hardware::EncoderDriver;

  EncoderDriver::Config cfg;
  cfg.port = "/dev/ttyAMA0";
  cfg.baud = 115200;
  cfg.de_re_pin = 26;
  cfg.gpio_chip = "/dev/gpiochip4";
  cfg.median_filter_size = 5;
  cfg.max_step_deg = 20.0;

  EncoderDriver enc;
  if (!enc.init(cfg)) {
    std::cerr << "init() failed" << std::endl;
    return 1;
  }

  // joints (mapping calibre 2026-05-30) : joint_idx 0-based
  enc.add_joint({0, 1, 16, 85, true, 0.0});        // joint_1 — motor 1, courroie 16/85, inverted
  enc.add_joint({1, 2, 15, 44, false, 0.0});       // joint_2 — motor 2, 15/44, non-inverted
  enc.add_joint({2, 3, 300, 1408, true, 0.0});     // joint_3 — motor 3, 300/1408, inverted

  // Charge zeros depuis le YAML package
  // (a executer apres `colcon build` qui installe le YAML dans share/)
  std::string yaml_path = "/home/roby/ros2_ws/install/roby_hardware/share/"
                          "roby_hardware/config/encoder_calibration.yaml";
  if (!enc.load_calibration_yaml(yaml_path)) {
    std::cerr << "load_calibration_yaml(" << yaml_path << ") failed" << std::endl;
    return 2;
  }

  // Couplage axe 2 -> 3 : joint_3 += joint_2 * (6000/45056) / (300/1408)
  const double m2 = 6000.0 / 45056.0;
  const double m3 = (15.0 * 20.0) / (44.0 * 32.0);
  enc.set_coupling(/*from=*/1, /*to=*/2, m2 / m3);

  // Boucle 50 Hz pendant 10s
  const auto period = std::chrono::microseconds(20000);  // 50 Hz
  const auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  auto next = std::chrono::steady_clock::now();
  auto last_print = std::chrono::steady_clock::now();
  int cycles = 0;

  while (std::chrono::steady_clock::now() < t_end) {
    next += period;
    enc.poll_all();
    cycles++;

    auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::seconds(1)) {
      auto j1 = enc.get_joint_position_rad(0);
      auto j2 = enc.get_joint_position_rad(1);
      auto j3 = enc.get_joint_position_rad(2);
      double freq = cycles / std::chrono::duration<double>(now - last_print).count();
      std::cout << "f=" << freq << "Hz "
                << " j1=" << (j1 ? j1.value() * 180.0 / M_PI : NAN) << "d"
                << " j2=" << (j2 ? j2.value() * 180.0 / M_PI : NAN) << "d"
                << " j3=" << (j3 ? j3.value() * 180.0 / M_PI : NAN) << "d"
                << " | outliers j1=" << enc.outliers_count(0)
                << " j2=" << enc.outliers_count(1)
                << " j3=" << enc.outliers_count(2)
                << std::endl;
      cycles = 0;
      last_print = now;
    }

    std::this_thread::sleep_until(next);
  }

  enc.shutdown();
  return 0;
}
