#ifndef PARTICLE_FILTER_ROBOT_H_
#define PARTICLE_FILTER_ROBOT_H_
#include <vector>
#include <random>

using namespace std;

const double landmarks[4][2] = {
    {20.0, 20.0}, {80.0, 80.0}, {20.0, 80.0}, {80.0, 20.0}};
const double world_size = 100;

//default_random_engine generator;
//uniform_real_distribution<double> uniform_distribution(0.0, 1.0);

static default_random_engine &get_engine() {
  // Initialized upon first call to the function.
  static default_random_engine engine;
  return engine;
}

static double uniform_distribution_sample() {
  // Initialized upon first call to the function.
  static uniform_real_distribution<double> uniform_distribution(0.0, 1.0);
  return uniform_distribution(get_engine());
}

class Robot {
 public:
  Robot();
  void set(double new_x, double new_y, double new_orientation);
  void set_noise(double f_noise, double t_noise, double s_noise);
  vector<double> sense();
  void move(double turn, double forward);
  double measurement_prob(std::vector<double> measurement);
  double eval(Robot r, std::vector<Robot> p, int counter);

//  getters and setters
  double get_x() const;
  void set_x(double x);
  double get_y() const;
  void set_y(double y);
  double get_orientation() const;
  void set_orientation(double orientation);
  double get_forward_noise() const;
  void set_forward_noise(double forward_noise);
  double get_turn_noise() const;
  void set_turn_noise(double turn_noise);
  double get_sense_noise() const;
  void set_sense_noise(double sense_noise);

 private:
  double x;
  double y;
  double orientation;
  double forward_noise;
  double turn_noise;
  double sense_noise;

};

double Gaussian(double mu, double sigma, double x);
double eval(Robot r, vector<Robot> p);
#endif