#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  // offsets into the vars array of the various state, error, and control elements
  static size_t x_start_;
  static size_t y_start_;
  static size_t psi_start_;
  static size_t v_start_;
  static size_t cte_start_;
  static size_t epsi_start_;
  static size_t delta_start_;
  static size_t a_start_;
  static double speed_target_;

  double steering_range_;
  double delta_q_;  // the previous steering value
  double a_q_;      // the previous acceleration value
};

#endif /* MPC_H */
