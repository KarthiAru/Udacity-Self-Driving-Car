#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include "grader.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * TODO: complete this function
 */
vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  VectorXd Si(3);
  Si << start[0], start[1], start[2];
  
  VectorXd Sf(3);
  Sf << end[0], end[1], end[2];
  
  // Matrix to solve for coefficients a3, a4, a5
  MatrixXd matrix_T(3, 3); 
  matrix_T << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
             3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
             6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);
  
  // Vector of boundary conditions for the equations
  VectorXd boundary_conditions(3);
  boundary_conditions << Sf[0] - (Si[0] + Si[1] * T + 0.5 * Si[2] * std::pow(T, 2)),
                         Sf[1] - (Si[1] + Si[2] * T),
                         Sf[2] - Si[2];

  // Solve the system of equations
  VectorXd alpha = matrix_T.inverse() * boundary_conditions;

  // Prepare the final result, which includes a0, a1, a2, a3, a4, a5
  vector<double> result = {Si[0], Si[1], 0.5 * Si[2]};
  for (int i = 0; i < alpha.size(); ++i) {
    result.push_back(alpha[i]);
  }
  
  return result;
  
}

int main() {

  // create test cases
  vector< test_case > tc = create_tests();

  bool total_correct = true;

  for(int i = 0; i < tc.size(); ++i) {
    vector<double> jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
    bool correct = close_enough(jmt, answers[i]);
    total_correct &= correct;
  }

  if(!total_correct) {
    std::cout << "Try again!" << std::endl;
  } else {
    std::cout << "Nice work!" << std::endl;
  }

  return 0;
}