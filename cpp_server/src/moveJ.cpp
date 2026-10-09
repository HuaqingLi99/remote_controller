#include "moveJ.h"

#include <array>
#include <cmath>
#include <functional>
#include <algorithm>
#include <stdexcept>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

/**
 * Single-target joint-space impedance motion.
 *
 * This controller generates a synchronized trapezoidal joint trajectory from
 * the current q to q_d. All joints share the same total motion time, and the
 * torque command tracks q/dq/ddq references with coriolis compensation.
 */

namespace {

struct TrajProfile {
  double T{0.0};                         // synchronized total time
  std::array<double, 7> ta{};            // accel time
  std::array<double, 7> tv{};            // constant vel time
  std::array<double, 7> vp{};            // peak vel
  std::array<double, 7> dir{};           // +1 / -1
  std::array<double, 7> q_start{};
  std::array<double, 7> q_goal{};
  std::array<double, 7> acc_max{};
};

struct RefSample {
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> ddq{};
};

TrajProfile make_synchronized_trapezoid(
    const std::array<double, 7>& q_start,
    const std::array<double, 7>& q_goal,
    const std::array<double, 7>& vmax,
    const std::array<double, 7>& acc_max) {
  TrajProfile profile;
  profile.q_start = q_start;
  profile.q_goal = q_goal;
  profile.acc_max = acc_max;

  double T = 0.0;

  // First pass: find the synchronized total time T
  for (size_t i = 0; i < 7; ++i) {
    const double dq_total = q_goal[i] - q_start[i];
    const double dq_abs = std::abs(dq_total);
    const double a = acc_max[i];
    const double v = vmax[i];

    if (a <= 0.0 || v <= 0.0) {
      throw std::runtime_error("vmax and acc_max must be positive");
    }

    profile.dir[i] = (dq_total >= 0.0) ? 1.0 : -1.0;

    if (dq_abs < 1e-12) {
      continue;
    }

    double t_ramp = v / a;
    double d_ramp = 0.5 * a * t_ramp * t_ramp;

    double t_total = 0.0;
    if (2.0 * d_ramp >= dq_abs) {
      // triangular
      t_ramp = std::sqrt(dq_abs / a);
      t_total = 2.0 * t_ramp;
    } else {
      // trapezoidal
      const double d_const = dq_abs - 2.0 * d_ramp;
      const double t_const = d_const / v;
      t_total = 2.0 * t_ramp + t_const;
    }

    T = std::max(T, t_total);
  }

  if (T < 1e-6) {
    T = 1e-3;
  }

  profile.T = T;

  // Second pass: solve each joint under this synchronized T
  for (size_t i = 0; i < 7; ++i) {
    const double dq_total = q_goal[i] - q_start[i];
    const double dq_abs = std::abs(dq_total);
    const double a = acc_max[i];

    if (dq_abs < 1e-12) {
      profile.ta[i] = 0.0;
      profile.tv[i] = T;
      profile.vp[i] = 0.0;
      continue;
    }

    double disc = T * T - 4.0 * dq_abs / a;
    if (disc < 0.0) {
      disc = 0.0;
    }

    const double ta = 0.5 * (T - std::sqrt(disc));
    const double tv = T - 2.0 * ta;
    const double vp = a * ta;

    if (vp > vmax[i] + 1e-9) {
      throw std::runtime_error("Synchronized trajectory exceeds vmax");
    }

    profile.ta[i] = ta;
    profile.tv[i] = tv;
    profile.vp[i] = vp;
  }

  return profile;
}

RefSample evaluate_reference(const TrajProfile& profile, double t) {
  RefSample sample{};

  
  const double tc = std::max(0.0, std::min(t, profile.T));

  for (size_t i = 0; i < 7; ++i) {
    const double q0 = profile.q_start[i];
    const double qg = profile.q_goal[i];
    const double dir = profile.dir[i];
    const double a = profile.acc_max[i];
    const double ta = profile.ta[i];
    const double tv = profile.tv[i];
    const double vp = profile.vp[i];
    const double dq_abs = std::abs(qg - q0);

    if (dq_abs < 1e-12) {
      sample.q[i] = q0;
      sample.dq[i] = 0.0;
      sample.ddq[i] = 0.0;
      continue;
    }

    double q_rel = 0.0;
    double dq_rel = 0.0;
    double ddq_rel = 0.0;

    if (tc <= ta) {
      // acceleration phase
      q_rel = 0.5 * a * tc * tc;
      dq_rel = a * tc;
      ddq_rel = a;
    } else if (tc <= ta + tv) {
      // constant velocity phase
      const double t2 = tc - ta;
      q_rel = 0.5 * a * ta * ta + vp * t2;
      dq_rel = vp;
      ddq_rel = 0.0;
    } else if (tc <= profile.T) {
      // deceleration phase
      const double t3 = tc - ta - tv;
      const double q_before = 0.5 * a * ta * ta + vp * tv;
      q_rel = q_before + vp * t3 - 0.5 * a * t3 * t3;
      dq_rel = std::max(0.0, vp - a * t3);
      ddq_rel = -a;
    }

    sample.q[i] = q0 + dir * q_rel;
    sample.dq[i] = dir * dq_rel;
    sample.ddq[i] = dir * ddq_rel;
  }

  // enforce exact final values at the end
  if (t >= profile.T) {
    sample.q = profile.q_goal;
    sample.dq.fill(0.0);
    sample.ddq.fill(0.0);
  }

  return sample;
}

}  

int moveJ(
    franka::Robot& robot,
    const std::array<double, 7>& q_d,
    const std::array<std::array<double, 7>, 7>& stiffness,
    const std::array<double, 7>& vmax,
    const std::array<double, 7>& acc_max,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<double()>& slowdown_factor_callback,
    const std::function<bool()>& should_stop) {
  franka::Model model = robot.loadModel();
  franka::RobotState initial_state = robot.readOnce();
  std::array<double, 7> q_start = initial_state.q;

  const TrajProfile profile =make_synchronized_trapezoid(q_start, q_d, vmax, acc_max);
  Eigen::Matrix<double, 7, 7> Kq = Eigen::Matrix<double, 7, 7>::Zero();
  Eigen::Matrix<double, 7, 7> Dq = Eigen::Matrix<double, 7, 7>::Zero();
  int zeta = 1;
  for (size_t i = 0; i < 7; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Kq(i, j) = stiffness[i][j];
    }
    Dq(i, i) = 2.0 * zeta * std::sqrt(std::max(Kq(i, i), 0.0));
  }

  double motion_time = 0.0;

  auto torque_callback =
      [&](const franka::RobotState& robot_state,
          franka::Duration duration) -> franka::Torques {
    const double dt = duration.toSec();

    double slowdown_factor = slowdown_factor_callback();
    if (!std::isfinite(slowdown_factor) || slowdown_factor < 1.0) {
      slowdown_factor = 1.0;
    }
    motion_time += dt / slowdown_factor;

    state_callback(robot_state);

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());


    RefSample ref = evaluate_reference(profile, motion_time);
    for (size_t i = 0; i < 7; ++i) {
      ref.dq[i] /= slowdown_factor;
      ref.ddq[i] /= (slowdown_factor * slowdown_factor);
    }
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_ref(ref.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_ref(ref.dq.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> ddq_ref(ref.ddq.data());





    Eigen::Matrix<double, 7, 1> q_error = q_ref - q;
    Eigen::Matrix<double, 7, 1> dq_error = dq_ref - dq;

    std::array<double, 49> mass_array = model.mass(robot_state);
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(mass_array.data());

    std::array<double, 7> coriolis_array = model.coriolis(robot_state);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());


    if (should_stop()) {
        std::array<double, 7> safe_tau{};
        for (size_t i = 0; i < 7; ++i) {
            safe_tau[i] = coriolis_array[i] - 2.0 * Dq(i, i) * robot_state.dq[i];
        }

        safe_tau = franka::limitRate(
            franka::kMaxTorqueRate,
            safe_tau,
            robot_state.tau_J_d
        );

        return franka::MotionFinished(franka::Torques(safe_tau));
    }


    Eigen::Matrix<double, 7, 1> tau_d =
        M * ddq_ref + coriolis + Kq * q_error + Dq * dq_error;

    std::array<double, 7> tau_d_array{};
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;


    tau_d_array = franka::limitRate(franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);


    if (motion_time >= profile.T) {
      std::cout << "moveJ finished at trajectory end." << std::endl;
      return franka::MotionFinished(franka::Torques(tau_d_array));
    }


    return franka::Torques(tau_d_array);
  };

  robot.control(torque_callback);
  std::cout << "Motion finished." << std::endl;
  return 0;
}
