#include <random>
#include <stdexcept>

#include "robot.h"

using namespace std;

Robot::Robot()
    : x(uniform_distribution_sample() * world_size),
      y(uniform_distribution_sample() * world_size),
      orientation(uniform_distribution_sample() * 2.0 * M_PI),
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
  set_x(new_x);
  set_y(new_y);
  set_orientation(new_orientation);
}

void Robot::set_noise(double f_noise, double t_noise, double s_noise) {
  set_forward_noise(f_noise);
  set_turn_noise(t_noise);
  set_sense_noise(s_noise);
}

vector<double> Robot::sense() {
  vector<double> Z;
  for (int i = 0; i < (sizeof landmarks / sizeof landmarks[0]); i++) {
    double dist = sqrt((get_x() - landmarks[i][0]) * (get_x() - landmarks[i][0]) +
        (get_y() - landmarks[i][1]) * (get_y() - landmarks[i][1]));
    normal_distribution<double> distribution(0.0, get_forward_noise());
    dist += distribution(get_engine());
    Z.push_back(dist);
  }
  return Z;
}

void Robot::move(double turn, double forward) {
  if (forward < 0) {
    throw invalid_argument("Robot cant move backwards");
  }
  normal_distribution<double> distribution1(0.0, get_turn_noise());
  normal_distribution<double> distribution2(0.0, get_forward_noise());
  orientation = orientation + turn + distribution1(get_engine());
  orientation = fmod(orientation, 2 * M_PI);
  if (orientation < 0) {
    orientation = 0.0;
  }
  double dist = forward + distribution2(get_engine());
  x = x + cos(orientation) * dist;
  y = y + sin(orientation) * dist;
  x = fmod(x, world_size);
  y = fmod(y, world_size);
  if (x < 0) {
    x = 0.0;
  }
  if (y < 0) {
    y = 0.0;
  }
}

double Robot::measurement_prob(vector<double> measurement) {
  double prob = 1.0;
  for (int i = 0; i < (sizeof landmarks / sizeof landmarks[0]); i++) {
    double dist = sqrt((get_x() - landmarks[i][0]) * (get_x() - landmarks[i][0]) +
        (get_y() - landmarks[i][1]) * (get_y() - landmarks[i][1]));
    prob *= Gaussian(dist, get_sense_noise(), measurement[i]);
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
    double dx = (p[i].get_x() - r.get_x() + fmod(world_size / 2.0, world_size) -
        (world_size / 2.0));
    double dy = (p[i].get_y() - r.get_y() + fmod(world_size / 2.0, world_size) -
        (world_size / 2.0));
    double err = sqrt(dx * dx + dy * dy);
    sum += err;
  }
  return sum / (double) p.size();
}


double Robot::get_x() const {
  return x;
}
void Robot::set_x(double x) {
  Robot::x = x;
}
double Robot::get_y() const {
  return y;
}
void Robot::set_y(double y) {
  Robot::y = y;
}
double Robot::get_orientation() const {
  return orientation;
}
void Robot::set_orientation(double orientation) {
  Robot::orientation = orientation;
}
double Robot::get_forward_noise() const {
  return forward_noise;
}
void Robot::set_forward_noise(double forward_noise) {
  Robot::forward_noise = forward_noise;
}
double Robot::get_turn_noise() const {
  return turn_noise;
}
void Robot::set_turn_noise(double turn_noise) {
  Robot::turn_noise = turn_noise;
}
double Robot::get_sense_noise() const {
  return sense_noise;
}
void Robot::set_sense_noise(double sense_noise) {
  Robot::sense_noise = sense_noise;
}
