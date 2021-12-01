#include <cstdlib>
#include <iostream>
#include <vector>
#include <chrono>

#include "robot.h"

int main(int argc, char *argv[]) {
  auto start = std::chrono::high_resolution_clock::now();

  if (argc == 2 || argc > 3) {
    cout << "Usage: ./particle_filter_cpu <optional:number_iteration "
            "(default:1000)> <optional:number_particles (default:10)>"
         << endl;
    exit(1);
  }

  // physical robot (ground truth)
  Robot myrobot;
  myrobot.set_x(world_size/2);
  myrobot.set_y(world_size/2);

  // list storing the distance of the physical robot to each obstacle
  vector<double> Z;

  // number of particles
  int N = 1000;

  // number of PF iterations
  int T = 10;

  if (argc == 3) {
    N = atoi(argv[1]);
    T = atoi(argv[2]);
  }

  // initialize particles
  vector<Robot> particles(N);

  // particle filter

  for (int j = 0; j < T; j++) {
    // make our physical robot move (theta, distance)
    double theta = uniform_distribution_sample() * M_PI / 2;
    double distance = (uniform_distribution_sample() * 9.0) + 1;
    myrobot.move(theta, distance);

    // detect its distance to the landmarks in our world (returns list of
    // distance to each obstacle)
    Z = myrobot.sense();

    vector<double> w(N);
    for (int k = 0; k < N; k++) {
      // make every particle do same movement as physical robot
      particles[k].move(theta, distance);

      // measure probability that the particle is the physical robot
      w[k] = particles[k].measurement_prob(Z);
    }

    // weight summation
    double weight_sum = 0.0;
    for (auto weight: w) weight_sum += weight;

    // weight normalization
    for (int k = 0; k < N; k++) {
      w[k] /= weight_sum;
    }

    // compute cumulative distribution function (CDF)
    vector<double> cdf(N);
    double running_sum = 0;
    for (int k = 0; k < N; k++) {
      cdf[k] = w[k] + running_sum;
      running_sum += w[k];
    }

    int k = 0;
    auto u = [&N](int n) {
      return (((n - 1) + uniform_distribution_sample()) / N);
    };

    // systematic_sampling resampling of new particles
    vector<Robot> new_particles;
    for (int i = 1; i <= N; i++) {
      while (cdf[k] < u(i)) {
        k += 1;
      }
      new_particles.push_back(particles[k]);
    }
    particles = new_particles;

    // evaluation - measure distance of all particles to the actual robot

    cout << eval(myrobot, particles, j) << endl;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::cout << N << "," << std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() << "\n";

  return 0;
}