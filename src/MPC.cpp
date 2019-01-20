#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// timestep length and duration
size_t N = 10;
double dt = 0.05;

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


// Reference velocity is set to 80 mph.
const double ref_v = 80.0;

// tuning parameter values for cost functions
const double cost_cte = 22.0;
const double cost_heading = 27.0;
const double cost_velocity = 5.0;
const double cost_steer_input = 1.0;
const double cost_accel_input = 10.0;
const double cost_steer_variation = 270.0;
const double cost_accel_variation = 0.0;

// 30mph solution: 30.0, 4.0, 1.0, 5.0, 1.0, 10.0, 100.0, 0.0 - Extremely stable speed control 29.3mph around whole track
// 40mph solution: 40.0, 6.0, 4.0, 5.0, 1.0, 10.0, 160.0, 0.0 - Extremely stable speed control 39.1mph around whole track, no erratic predictions created
// 50mph solution: 50.0, 14.0, 21.0, 5.0, 1.0, 10.0, 270.0, 0.0 - Extremely stable speed control 48.89mph around whole track, no erratic predictions created
// 60mph solution: 60.0, 22.0, 27.0, 5.0, 1.0, 10.0, 180.0, 0.0 - Extremely stable speed control 58.6mph around whole track, no erratic predictions created
// good for 70mph:extremely stable 67mph     80mph: extremely stable 76mph


// 70mph solution: 70.0, 70.0, 75.0, 1.5, 1.3, 4.8, 150.0, 0.0 - fairly stable speed control 67.8mph around most of track, some erratic predictions causes braking - vehicle manages to stay within road boundaries
// 70mph solution: 70.0, 500.0, 500.0, 1.5, 1.3, 4.8, 1000.0, 0.0 - stable speed control ~67mph around most of track, some erratic predictions causes braking, high cte, heading and steer force speed to dip on bends - vehicle manages to stay within road boundaries
// 80mph solution: 80.0, 600.0, 600.0, 1.5, 1.3, 4.8, 1200.0, 0.0 -  speed control ~76mph along straight, bends cause braking down to ~ 55mph, high cte, heading and steer force speed to dip on bends - vehicle manages to stay within road boundaries, prediction trajectory is quite stable.
// 200mph solution: 200.0, 12000.0, 12000.0, 1.5, 1.3, 4.8, 20000.0, 0.0 -  speed control ~91mph along straight, bends cause significant braking down to ~ 50mph, high cte, heading and steer force speed to dip on bends - vehicle manages to stay within road boundaries, prediction trajectory is very stable.

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
// set indexing of each
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // Cost Function
  void operator()(ADvector& fg, const ADvector& vars) {
  // The cost is stored is the first element of `fg`.
  // Any additions to the cost should be added to `fg[0]`.
  fg[0] = 0;

  // Cost functions - state values
  for (int t = 0; t < N; t++) {
      // cross track error cost
      fg[0] += cost_cte * CppAD::pow(vars[cte_start + t], 2);
      // desired angle difference cost - heading error
      fg[0] += cost_heading * CppAD::pow(vars[epsi_start + t], 2);
      // target velocity difference cost
      fg[0] += cost_velocity * CppAD::pow(vars[v_start + t] - ref_v, 2);
  }

  // Cost Functions to restict the change amount of the actuators.
  for (int t = 0; t < N - 1; t++) {
      // Penalty for too large steering input - smooth turn
      fg[0] += cost_steer_input * CppAD::pow(vars[delta_start + t], 2);
      // Penalty for too large acceleration - smooth accel
      fg[0] += cost_accel_input * CppAD::pow(vars[a_start + t], 2);
  }

  // Cost Functions to restict the change variance of the control inputs.
  // smooth transistions between control inputs - last one should be similar to next one
  for (int t = 0; t < N - 2; t++) {
      // modify steering sequential by value >1 (100 smooth, 500 even smoother) to keep steering values closer together
      fg[0] += cost_steer_variation * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      // acceleration change cost function
      fg[0] += cost_accel_variation * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  }

  // setup of constraints
  // Add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
  // This bumps up the position of all the other values.
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + psi_start] = vars[psi_start];
  fg[1 + v_start] = vars[v_start];
  fg[1 + cte_start] = vars[cte_start];
  fg[1 + epsi_start] = vars[epsi_start];

      // The rest of the constraints
      for (int t = 1; t < N; t++) {

          // state at time t+1
          AD<double> x1 = vars[x_start + t];  // x
          AD<double> y1 = vars[y_start + t];  // y
          AD<double> psi1 = vars[psi_start + t]; // angle
          AD<double> v1 = vars[v_start + t]; // velocity
          AD<double> cte1 = vars[cte_start + t]; // cte
          AD<double> epsi1 = vars[epsi_start + t]; // delta of desired angle

          // state at time 0
          AD<double> x0 = vars[x_start + t - 1]; // x
          AD<double> y0 = vars[y_start + t - 1];  // y
          AD<double> psi0 = vars[psi_start + t - 1];  // angle
          AD<double> v0 = vars[v_start + t - 1]; // velocity
          AD<double> cte0 = vars[cte_start + t - 1];  // cte
          AD<double> epsi0 = vars[epsi_start + t - 1];  // delta of desired angle

          // actuation only considered at time t
          AD<double> delta0 = vars[delta_start + t - 1]; // steering angle
          AD<double> a0 = vars[a_start + t - 1];  // acceleration


          // used to find the current cte - coeffs[0] = points x , coeffs[1] = points y
          AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
          // used to calculate the current orientatation error - coeffs[1] = points y
          AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

          //
          // NOTE: The use of `AD<double>` and use of `CppAD`!
          // This is also CppAD can compute derivatives and pass
          // these to the solver.

          // The idea here is to constraint this value to be 0.
          // model constraints
          fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt); // state x
          fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt); // state y
          fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);  // state psi
          fg[1 + v_start + t] = v1 - (v0 + a0 * dt); // state velocity
          fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)); // state cte
          fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt); // state epsi

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
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10(N) + 2 * 9(N-1)
  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = 6 * N  + 2 * (N - 1) ;;
  // Set the number of constraints
  size_t n_constraints = N * 6;
    
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

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  
  // Set all states upper and lower limits
  // allow all states to have any value setting to the max negative and positive values.
  for ( int i = 0; i < delta_start; i++ ) {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta(steering angle) are set to -25 and 25 degrees.
  // 25deg/180 * PI = 0.436332 RAD
  for (int i = delta_start; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
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

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
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
  // auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  // Return steering and acceleration actuator values
  double steer_value = solution.x[delta_start];
  double throttle_value = solution.x[a_start];
  
  // create a 2 element vector
  vector<double> mpc_result = {steer_value, throttle_value};

  // attach prediction of route to output
  for (int i=0; i<N; i++) {
      mpc_result.push_back(solution.x[x_start+i]);
      mpc_result.push_back(solution.x[y_start+i]);
  }
  // return [steering, acceleration, predicted_trajectory_x1, predicted_trajectory_y1, .... predicted_trajectory_xn, predicted_trajectory_yn]
  return mpc_result;
}
