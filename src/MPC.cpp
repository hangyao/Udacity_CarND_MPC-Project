#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.1;

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

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 100;

const int x_1st_index = 0;
const int y_1st_index = x_1st_index + N;
const int psi_1st_index = y_1st_index + N;
const int v_1st_index = psi_1st_index + N;
const int cte_1st_index = v_1st_index + N;
const int epsi_1st_index = cte_1st_index + N;
const int delta_1st_index = epsi_1st_index + N;
const int a_1st_index = delta_1st_index + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;
    const int cte_coeffs = 5000;
    const int epsi_coeffs = 1000;
    const int steering_coeffs = 100000;
    const int throttle_coeffs = 75;
    const int delta_coeffs = 100;
    const int a_coeffs = 10;
    for (int i = 0; i < N; i++) {
      fg[0] += cte_coeffs * CppAD::pow(vars[cte_1st_index + i] - ref_cte, 2);
      fg[0] += epsi_coeffs * CppAD::pow(vars[epsi_1st_index + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_1st_index + i] - ref_v, 2);
      if (i < N - 1) {
        fg[0] += steering_coeffs * CppAD::pow(vars[delta_1st_index + i], 2);
        fg[0] += throttle_coeffs * CppAD::pow(vars[a_1st_index + i], 2);
        if (i < N -2) {
          fg[0] += delta_coeffs * CppAD::pow(vars[delta_1st_index + i + 1] - vars[delta_1st_index + i], 2);
          fg[0] += a_coeffs * CppAD::pow(vars[a_1st_index + i + 1] - vars[a_1st_index + i], 2);
        }
      }
    }

    fg[1 + x_1st_index] = vars[x_1st_index];
    fg[1 + y_1st_index] = vars[y_1st_index];
    fg[1 + psi_1st_index] = vars[psi_1st_index];
    fg[1 + v_1st_index] = vars[v_1st_index];
    fg[1 + cte_1st_index] = vars[cte_1st_index];
    fg[1 + epsi_1st_index] = vars[epsi_1st_index];
    for (int i = 0; i < N - 1; i++) {
      AD<double> x_0 = vars[x_1st_index + i];
      AD<double> y_0 = vars[y_1st_index + i];
      AD<double> psi_0 = vars[psi_1st_index + i];
      AD<double> v_0 = vars[v_1st_index + i];
      AD<double> cte_0 = vars[cte_1st_index + i];
      AD<double> epsi_0 = vars[epsi_1st_index + i];
      AD<double> delta_0 = vars[delta_1st_index + i];
      AD<double> a_0 = vars[a_1st_index + i];
      AD<double> f_0 = coeffs[0] + coeffs[1] * x_0 + coeffs[2] * x_0 * x_0 + coeffs[3] * x_0 * x_0 * x_0;
      AD<double> psi_desired_0 = CppAD::atan(3 * coeffs[3] * x_0 * x_0 + 2 * coeffs[2] * x_0 + coeffs[1]);
      AD<double> x_1 = vars[x_1st_index + i + 1];
      AD<double> y_1 = vars[y_1st_index + i + 1];
      AD<double> psi_1 = vars[psi_1st_index + i + 1];
      AD<double> v_1 = vars[v_1st_index + i + 1];
      AD<double> cte_1 = vars[cte_1st_index + i + 1];
      AD<double> epsi_1 = vars[epsi_1st_index + i + 1];
      fg[2 + x_1st_index + i] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      fg[2 + y_1st_index + i] = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      fg[2 + psi_1st_index + i] = psi_1 - (psi_0 - v_0 * delta_0 / Lf * dt);
      fg[2 + v_1st_index + i] = v_1 - (v_0 + a_0 * dt);
      fg[2 + cte_1st_index + i] = cte_1 - ((f_0 - y_0) + (v_0 * CppAD::sin(epsi_0) * dt));
      fg[2 + epsi_1st_index + i] = epsi_1 - ((psi_0 - psi_desired_0) - v_0 * delta_0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  // size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  const int n_vars = N * 6 + (N - 1) * 2;
  const int n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < delta_1st_index; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (int i = delta_1st_index; i < a_1st_index; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for (int i = a_1st_index; i < n_vars; i++) {
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

  constraints_lowerbound[x_1st_index] = x;
  constraints_lowerbound[y_1st_index] = y;
  constraints_lowerbound[psi_1st_index] = psi;
  constraints_lowerbound[v_1st_index] = v;
  constraints_lowerbound[cte_1st_index] = cte;
  constraints_lowerbound[epsi_1st_index] = epsi;
  constraints_upperbound[x_1st_index] = x;
  constraints_upperbound[y_1st_index] = y;
  constraints_upperbound[psi_1st_index] = psi;
  constraints_upperbound[v_1st_index] = v;
  constraints_upperbound[cte_1st_index] = cte;
  constraints_upperbound[epsi_1st_index] = epsi;

  // object that computes objective and constraints
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
  std::cout << "Cost " << cost << std::endl;

  vector<double> mpc_solution;
  mpc_solution.push_back(solution.x[delta_1st_index]);
  mpc_solution.push_back(solution.x[a_1st_index]);
  for (int i = 0; i < N - 1; i++) {
  	mpc_solution.push_back(solution.x[x_1st_index + i + 1]);
  	mpc_solution.push_back(solution.x[y_1st_index + i + 1]);
  }
  return mpc_solution;
}
