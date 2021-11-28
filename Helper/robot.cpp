#include <random>
#include <stdexcept>

#include "robot.h"

using namespace std;

Robot::Robot()
    : x(uniform_distribution(generator) * world_size),
      y(uniform_distribution(generator) * world_size),
      orientation(uniform_distribution(generator) * 2.0 * M_PI),
      forward_noise(0.05), 
      turn_noise(0.05),
      sense_noise(5.0) {}

void Robot::set(double new_x, double new_y, double new_orientation) {
  if (new_x < 0 || new_x >= world_size) {
    throw invalid_argument("X coordinate out of bound");
  }
  if (new_y < 0 || new_y >= world_size) {
    throw invalid_argument("Y coordinate out of bound");
  }
  if (new_orientation < 0 || new_orientation >= 2 * M_PI) {
    throw invalid_argument("Orientation must be in [0..2pi]");
  }
  x = new_x;
  y = new_y;
  orientation = new_orientation;
}

void Robot::set_noise(double f_noise, double t_noise, double s_noise) {
  forward_noise = f_noise;
  turn_noise = t_noise;
  sense_noise = s_noise;
}

vector<double> Robot::sense() {
  vector<double> Z;
  for (int i = 0; i < (sizeof landmarks / sizeof landmarks[0]); i++) {
    double dist = sqrt((x - landmarks[i][0]) * (x - landmarks[i][0]) +
                       (y - landmarks[i][1]) * (y - landmarks[i][1]));
    normal_distribution<double> distribution(0.0, forward_noise);
    dist += distribution(generator);
    Z.push_back(dist);
  }
  return Z;
}

Robot Robot::move(double turn, double forward) {
  if (forward < 0) {
    throw invalid_argument("Robot cant move backwards");
  }
  normal_distribution<double> distribution1(0.0, turn_noise);
  normal_distribution<double> distribution2(0.0, forward_noise);
  double new_orientation = orientation + turn + distribution1(generator);
  new_orientation = fmod(new_orientation, 2 * M_PI);
  if (new_orientation < 0) {
    new_orientation = 0.0;
  }
  double dist = forward + distribution2(generator);
  double new_x = x + cos(orientation) * dist;
  double new_y = y + sin(orientation) * dist;
  new_x = fmod(new_x, world_size);
  new_y = fmod(new_y, world_size);
  if (new_x < 0) {
    new_x = 0.0;
  }
  if (new_y < 0) {
    new_y = 0.0;
  }
  Robot res;
  res.set(new_x, new_y, new_orientation);
  res.set_noise(forward_noise, turn_noise, sense_noise);
  return res;
}

double Robot::measurement_prob(vector<double> measurement) {
  double prob = 1.0;
  for (int i = 0; i < (sizeof landmarks / sizeof landmarks[0]); i++) {
    double dist = sqrt((x - landmarks[i][0]) * (x - landmarks[i][0]) +
                       (y - landmarks[i][1]) * (y - landmarks[i][1]));
    prob *= Gaussian(dist, sense_noise, measurement[i]);
  }
  return prob;
}

double Gaussian(double mu, double sigma, double x) {
  return exp(-((mu - x) * (mu - x)) / (sigma * sigma) / 2.0) /
         sqrt(2.0 * M_PI * (sigma * sigma));
}

double eval(Robot r, vector<Robot> p) {
  double sum = 0.0;
  for (int i = 0; i < p.size(); i++) {
    double dx = (p[i].x - r.x + fmod(world_size / 2.0, world_size) -
                 (world_size / 2.0));
    double dy = (p[i].y - r.y + fmod(world_size / 2.0, world_size) -
                 (world_size / 2.0));
    double err = sqrt(dx * dx + dy * dy);
    sum += err;
  }
  return sum / (double)p.size();
}
