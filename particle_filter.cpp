#include <cstdlib>
#include <iostream>
#include <vector>
#include <chrono>

#include "robot.h"

int main(int argc, char *argv[]) {
  auto start = std::chrono::high_resolution_clock::now();

  if (argc == 2 || argc > 3) {
    cout
        << "Usage: ./particle_filter_cpu <optional:number_iteration (default:1000)> <optional:number_particles (default:10)>"
        << endl;
    exit(1);
  }

  // physical robot (ground truth)
  Robot myrobot;
  
  // list storing the distance of the physical robot to each obstacle
  std::vector<double> Z;
  
  // number of particles
  int N = 1000;

  // number of PF iterations
  int T = 10;
  if (argc == 3) {
    N = atoi(argv[1]);
    T = atoi(argv[2]);
  }

  // initialize particles
  std::vector<Robot> p;
  for (int i = 0; i < N; i++) {
    Robot r;
    r.set_noise(0.05, 0.05, 5.0);
    p.push_back(r);
  }

  // particle filter

  for (int j = 0; j < T; j++) {

    // make our physical robot move (theta, distance)
    double theta = uniform_distribution_sample() * M_PI/2;
    double distance = (uniform_distribution_sample() * 9.0)+1;
    myrobot.move(theta, distance);

    // detect its distance to the landmarks in our world (returns list of distance to each obstacle)
    Z = myrobot.sense();

    // make every particle do same movement as physical robot
    for (int k = 0; k < N; k++) {
      p[k].move(theta, distance);
    }

    // measure probability that each particle is the physical robot
    std::vector<double> w;
    for (int k = 0; k < N; k++) {
      w.push_back(p[k].measurement_prob(Z));
    }

    std::vector<Robot> p3;
    int index = (int) (uniform_distribution_sample() * N);
    double beta = 0.0;
    double max_w = w[0];
    for (int l = 0; l < w.size(); l++) {
      if (w[l] > max_w) {
        max_w = w[l];
      }
    }

    // resampling
    for (int m = 0; m < N; m++) {
      beta += uniform_distribution_sample() * 2.0 * max_w;
      while (beta > w[index]) {
        beta -= w[index];
        index = (index + 1) % N;
      }
      p3.push_back(p[index]);
    }
    p = p3;

    // evaluation - measure distance of all particles to the actual robot
//    cout << eval(myrobot, p) << endl;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::cout << N <<","<<std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() << "\n";

  return 0;
}