#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

#include "robot.h"

int main() {
  Robot myrobot;
  myrobot = myrobot.move(0.1, 5.0);
  std::vector<double> Z = myrobot.sense();
  int N = 1000;
  int T = 10;

  std::vector<Robot> p;
  for (int i = 0; i < N; i++) {
    Robot r;
    r.set_noise(0.05, 0.05, 5.0);
    p.push_back(r);
  }

  for (int j = 0; j < T; j++) {
    myrobot = myrobot.move(0.1, 5.0);
    Z = myrobot.sense();

    std::vector<Robot> p2;
    for (int k = 0; k < N; k++) {
      p2.push_back(p[k].move(0.1, 5.0));
    }
    p = p2;

    std::vector<double> w;
    for (int k = 0; k < N; k++) {
      w.push_back(p[k].measurement_prob(Z));
    }

    std::vector<Robot> p3;
    int index = (int)(uniform_distribution_sample() * N);
    double beta = 0.0;
    double max_w = w[0];
    for (int l = 0; l < w.size(); l++) {
      if (w[l] > max_w) {
        max_w = w[l];
      }
    }

    for (int m = 0; m < N; m++) {
      beta += uniform_distribution_sample()  * 2.0 * max_w;
      while (beta > w[index]) {
        beta -= w[index];
        index = (index + 1) % N;
      }
      p3.push_back(p[index]);
    }
    p = p3;

    cout << eval(myrobot, p) << endl;
  }

  return 0;
}