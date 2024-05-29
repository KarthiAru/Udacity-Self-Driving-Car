#include <iostream>
#include <vector>
#include <algorithm> // Include for std::fill

using std::vector;

// Initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev);

int main() {
  // Set standard deviation of position
  float position_stdev = 1.0f;

  // Set map horizon distance in meters 
  int map_size = 25;

  // Initialize landmarks
  vector<float> landmark_positions {5, 10, 20};

  // Initialize priors
  vector<float> priors = initialize_priors(map_size, landmark_positions, position_stdev);

  // Print values to stdout 
  for (float prior : priors) {
    std::cout << prior << std::endl;
  }

  return 0;
}

// Implement the initialize_priors function
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev) {

  // Initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev

  // Set all priors to 0.0
  vector<float> priors(map_size, 0.0);
    
  float probability = 1.0 / (landmark_positions.size() * (2 * position_stdev + 1));

  // Set priors for each landmark position considering position_stdev
  for (float position : landmark_positions) {
    int start = std::max(0, static_cast<int>(position - position_stdev));
    int end = std::min(map_size, static_cast<int>(position + position_stdev + 1)); // +1 because std::fill does not include the end position
    std::fill(priors.begin() + start, priors.begin() + end, probability);
  }

  return priors;
}
