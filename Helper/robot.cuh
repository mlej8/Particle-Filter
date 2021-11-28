#ifndef PARTICLE_FILTER_ROBOT_H_
#define PARTICLE_FILTER_ROBOT_H_
#include <vector>
#include <random>

using namespace std;

const double landmarks[4][2] = {
    {20.0, 20.0}, {80.0, 80.0}, {20.0, 80.0}, {80.0, 20.0}};
const double world_size = 100;

static default_random_engine &get_engine();
static double uniform_distribution_sample();

class Robot {
 public:
  Robot();
  __host__ void set(double new_x, double new_y, double new_orientation);
  __host__ void set_noise(double f_noise, double t_noise, double s_noise);
  __host__ vector<double> sense();
  __host__ __device__ void move(double turn, double forward);
  __host__ double measurement_prob(std::vector<double> measurement);
  __host__ __device__ double eval(Robot r, std::vector<Robot> p, int counter);

  // getters and setters
  __host__ __device__ double get_x() const;
  __host__ __device__ void set_x(double x);
  __host__ __device__ double get_y() const;
  __host__ __device__ void set_y(double y);
  __host__ __device__ double get_orientation() const;
  __host__ __device__ void set_orientation(double orientation);
  __host__ __device__ double get_forward_noise() const;
  __host__ __device__ void set_forward_noise(double forward_noise);
  __host__ __device__ double get_turn_noise() const;
  __host__ __device__ void set_turn_noise(double turn_noise);
  __host__ __device__ double get_sense_noise() const;
  __host__ __device__ void set_sense_noise(double sense_noise);

 private:
  double x;
  double y;
  double orientation;
  double forward_noise;
  double turn_noise;
  double sense_noise;

};

__host__ __device__ double Gaussian(double mu, double sigma, double x);
__host__ double eval(Robot r, vector<Robot> p);
#endif