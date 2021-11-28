#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "robot.h"

using namespace std;

__global__ void simulate_robot_motion() {}

__global__ void importance_weight_computation() {}

__global__ void resampling() {}

int main() {
  // physical robot (ground truth)
  Robot my_robot;

  // number of particles
  int N = 1000;

  // number of iterations of particle filter
  int T = 10;

  // initialize N random particles (robots)
  // list of particles  (guesses as to where the robot might be - each particle
  // is a vector representing the state of the robot (x,y,theta)  theta is the
  // angle relative to the x-axis)
  std::vector<Robot> particles;
  for (int i = 0; i < N; i++) {
    Robot r;
    r.set_noise(0.05, 0.05, 5.0);
    particles.push_back(r);
  }

  for (int j = 0; j < T; j++) {
    // TODO here we are always turning by 0.1 and moving for 5 meters, randomly
    // generate the movement of the True robot
    my_robot = my_robot.move(0.1, 5.0);
    vector<double> Z = my_robot.sense();

    // TODO take each of the particles and simulate robot motion
    std::vector<Robot> p2;
    for (int k = 0; k < N; k++) {
      p2.push_back(particles[k].move(0.1, 5.0));
    }
    particles = p2;

    // TODO weigth computation
    std::vector<double> w;
    for (int k = 0; k < N; k++) {
      w.push_back(particles[k].measurement_prob(Z));
    }

    std::vector<Robot> p3;

    // resampling
    int index = (int)(uniform_distribution(generator) * N);
    double beta = 0.0;
    double max_w = w[0];
    for (int l = 0; l < w.size(); l++) {
      if (w[l] > max_w) {
        max_w = w[l];
      }
    }

    for (int m = 0; m < N; m++) {
      beta += uniform_distribution(generator) * 2.0 * max_w;
      while (beta > w[index]) {
        beta -= w[index];
        index = (index + 1) % N;
      }
      p3.push_back(particles[index]);
    }
    particles = p3;

    cout << eval(my_robot, particles) << endl;
  }
  return 0;
}