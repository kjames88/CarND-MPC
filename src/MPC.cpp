#define HAVE_CSTDDEF 1

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// timestep length and duration
size_t N = 25;
double dt = 0.05;

size_t MPC::x_start_ = 0;
size_t MPC::y_start_ = x_start_ + N;
size_t MPC::psi_start_ = y_start_ + N;
size_t MPC::v_start_ = psi_start_ + N;
size_t MPC::cte_start_ = v_start_ + N;
size_t MPC::epsi_start_ = cte_start_ + N;
size_t MPC::delta_start_ = epsi_start_ + N;
size_t MPC::a_start_ = delta_start_ + N - 1;
double MPC::speed_target_ = 50.0;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  AD<double> cte(AD<double> x, AD<double> y, AD<double> x0, AD<double> f0, AD<double> fprime0) {
    // compute the normal distance from (x,y) to the tangent of the line specified by coeffs at x0
    //   fprime0 is the slope at (x0,f0)    
    //   for line with slope m, at point (x,y), d = abs(mx - y + b) / sqrt(m**2 + 1) (adapted from Anton p138)
    //   projecting back to x=0:  y0 - mx0 = b
    AD<double> b = f0 - (fprime0 * x0);
    //std::cout << "x=" << x << " y=" << y << " x0=" << x0 << " f0=" << f0 << " fprime0=" << fprime0 << " b=" << b << std::endl;    
    return CppAD::abs(fprime0 * x - y + b) / CppAD::sqrt(CppAD::pow(fprime0,2) + 1);
  }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // fg[0] contains cost

    // state (N terms)
    for (int i=0; i<N; i++) {
      // position and orientation errors
      fg[0] += 0.1 * CppAD::pow(vars[MPC::cte_start_ + i], 2);
      fg[0] += CppAD::pow(vars[MPC::epsi_start_ + i], 2);
      // speed regulation
      fg[0] += CppAD::pow(vars[MPC::v_start_ + i] - AD<double> (MPC::speed_target_), 2);
    }

    // control (N-1 terms)
    for (int i=0; i<N-1; i++) {
      // penalize sharp steering and high throttle/brake
      fg[0] += CppAD::pow(vars[MPC::delta_start_ + i], 2);
      fg[0] += CppAD::pow(vars[MPC::a_start_ + i], 2);
    }

    // change in control (N-2 terms)
    for (int i=0; i<N-2; i++) {
      // try to keep the changes in control inputs smooth
      fg[0] += 500.0 * CppAD::pow(vars[MPC::delta_start_ + i + 1] - vars[MPC::delta_start_ + i], 2);
      fg[0] += 25.0 * CppAD::pow(vars[MPC::a_start_ + i + 1] - vars[MPC::a_start_ + i], 2);
    }

    // The first step constraint is the current state
    //   Offset of 1 due to fg[0] used for cost.
    fg[1 + MPC::x_start_] = vars[MPC::x_start_];
    fg[1 + MPC::y_start_] = vars[MPC::y_start_];
    fg[1 + MPC::psi_start_] = vars[MPC::psi_start_];
    fg[1 + MPC::v_start_] = vars[MPC::v_start_];
    fg[1 + MPC::cte_start_] = vars[MPC::cte_start_];
    fg[1 + MPC::epsi_start_] = vars[MPC::epsi_start_];
    
    // Constraints for the remainder of steps
    //   Offset of 1 because fg[0] contains cost
    for (int t=1; t<N; t++) {
      AD<double> x0 = vars[MPC::x_start_ + t - 1];
      AD<double> y0 = vars[MPC::y_start_ + t - 1];
      AD<double> psi0 = vars[MPC::psi_start_ + t - 1];
      AD<double> v0 = vars[MPC::v_start_ + t - 1];
      AD<double> cte0 = vars[MPC::cte_start_ + t - 1];
      AD<double> epsi0 = vars[MPC::epsi_start_ + t - 1];
      AD<double> delta0 = vars[MPC::delta_start_ + t - 1];
      AD<double> a0 = vars[MPC::a_start_ + t - 1];

      AD<double> x1 = vars[MPC::x_start_ + t];
      AD<double> y1 = vars[MPC::y_start_ + t];
      AD<double> psi1 = vars[MPC::psi_start_ + t];
      AD<double> v1 = vars[MPC::v_start_ + t];
      AD<double> cte1 = vars[MPC::cte_start_ + t];
      AD<double> epsi1 = vars[MPC::epsi_start_ + t];
      // 3rd degree polynomial fit
      AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> fprime0 = coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 * coeffs[3] * CppAD::pow(x0, 2);

      // The basic form here is x1 - (x0 + v cos(psi) dt) = 0 due to the min/max constraints set to 0 in Solve()
      //   Offset of 1 due to fg[0] used for cost.  Another offset of 1 because the first step is populated above (current state)
      AD<double> _dt = dt;
      fg[1 + MPC::x_start_ + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * _dt);
      fg[1 + MPC::y_start_ + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * _dt);
      fg[1 + MPC::psi_start_ + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * _dt);
      fg[1 + MPC::v_start_ + t] = v1 - (v0 + a0 * _dt);
      fg[1 + MPC::epsi_start_ + t] = epsi1 - ((psi0 - CppAD::atan(fprime0)) + (v0/Lf) * delta0 * _dt);
      fg[1 + MPC::cte_start_ + t] = cte1 - cte(x0 + v0 * CppAD::cos(epsi0) * _dt, y0 + v0 * CppAD::sin(epsi0), x0, f0, fprime0);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  
  // 4 state variables (x,y,psi,v) * N steps
  // 2 error variables (cte, epsi) * N steps
  // 2 control inputs (delta, a) * (N-1) steps
  size_t n_vars = (6 * N) + (2 * (N-1));
  // the number of constraints (6 per step:  x,y,psi,v,cte,epsi)
  size_t n_constraints = 6 * N;

  // State at the time of this invocation
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[x_start_] = x;
  vars[y_start_] = y;
  vars[psi_start_] = psi;
  vars[v_start_] = v;
  vars[cte_start_] = cte;
  vars[epsi_start_] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.

  // non actuators
  for (int i=0; i<delta_start_; i++) {
    // seems a bit excessive; following the quiz on this
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // actuators
  // steering
  double max_steering_angle = 25.0;  // degrees +/- steering range
  double range = (M_PI / 180.0) * max_steering_angle;
  steering_range_ = range;
  for (int i=delta_start_; i<a_start_; i++) {
    vars_lowerbound[i] = -range;
    vars_upperbound[i] = range;
  }

  // acceleration
  for (int i=a_start_; i<n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start_] = x;
  constraints_lowerbound[y_start_] = y;
  constraints_lowerbound[psi_start_] = psi;
  constraints_lowerbound[v_start_] = v;
  constraints_lowerbound[cte_start_] = cte;
  constraints_lowerbound[epsi_start_] = epsi;

  constraints_upperbound[x_start_] = x;
  constraints_upperbound[y_start_] = y;
  constraints_upperbound[psi_start_] = psi;
  constraints_upperbound[v_start_] = v;
  constraints_upperbound[cte_start_] = cte;
  constraints_upperbound[epsi_start_] = epsi;

  // object that computes objective and constraints
  //   coeffs contains coefficients of line fit to waypoints at the time of this invocation
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << " cte " << solution.x[cte_start_ + 1] << " epsi " << solution.x[epsi_start_ + 1]
            << " steer " << solution.x[delta_start_ + 1] << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> rval;
  rval.push_back(solution.x[delta_start_ + 1]);
  rval.push_back(solution.x[a_start_ + 1]);
  for (int i=2; i<N; i++) {
    rval.push_back(solution.x[x_start_ + i]);
    rval.push_back(solution.x[y_start_ + i]);
  }
  
  return rval;
}
