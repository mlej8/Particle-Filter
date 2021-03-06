#include <curand.h>
#include <curand_kernel.h>
#include <thrust/random.h>

#include <random>
#include <stdexcept>

#include "robot.cuh"

using namespace std;

#if defined(IMAGES)
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
using namespace cv;
#endif

__host__ default_random_engine &get_engine() {
  // Initialized upon first call to the function.
  static default_random_engine engine;
  return engine;
}

__host__ double uniform_distribution_sample() {
  // Initialized upon first call to the function.
  static uniform_real_distribution<double> uniform_distribution(0.0, 1.0);
  return uniform_distribution(get_engine());
}

Robot::Robot()
    : x(uniform_distribution_sample() * world_size),
      y(uniform_distribution_sample() * world_size),
      orientation(uniform_distribution_sample() * 2.0 * M_PI),
      forward_noise(0.05),
      turn_noise(0.05),
      sense_noise(5.0) {}

__host__ void Robot::set(double new_x, double new_y, double new_orientation) {
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

__host__ void Robot::set_noise(double f_noise, double t_noise, double s_noise) {
  set_forward_noise(f_noise);
  set_turn_noise(t_noise);
  set_sense_noise(s_noise);
}

__host__ vector<double> Robot::sense() {
  vector<double> Z;
  for (int i = 0; i < (sizeof landmarks / sizeof landmarks[0]); i++) {
    double dist =
        sqrt((get_x() - landmarks[i][0]) * (get_x() - landmarks[i][0]) +
            (get_y() - landmarks[i][1]) * (get_y() - landmarks[i][1]));
    normal_distribution<double> distribution(0.0, get_forward_noise());
    dist += distribution(get_engine());
    Z.push_back(dist);
  }
  return Z;
}

__host__ __device__ void Robot::move(double turn, double forward) {
  if (forward < 0) {
    // throw invalid_argument("Robot cant move backwards");
    forward = 0;
  }

  //  TODO we can make a  random generator from different seeds
  thrust::default_random_engine gen;
  //  see
  //  https://thrust.github.io/doc/classthrust_1_1random_1_1normal__distribution_a0c00f49ae6a01e0997e6871a054c61cc.html#a0c00f49ae6a01e0997e6871a054c61cc
  thrust::random::normal_distribution<double> distribution1(0.5,
                                                            get_turn_noise());
  thrust::random::normal_distribution<double> distribution2(
      0.5, get_forward_noise());

  orientation = orientation + turn + distribution1(gen);
  orientation = fmod(orientation, 2 * M_PI);
  if (orientation < 0) {
    orientation = 0.0;
  }
  double dist = forward + distribution2(gen);
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

__host__ double Robot::measurement_prob(vector<double> measurement) {
  double prob = 1.0;
  for (int i = 0; i < (sizeof landmarks / sizeof landmarks[0]); i++) {
    double dist =
        sqrt((get_x() - landmarks[i][0]) * (get_x() - landmarks[i][0]) +
            (get_y() - landmarks[i][1]) * (get_y() - landmarks[i][1]));
    prob *= Gaussian(dist, get_sense_noise(), measurement[i]);
  }
  return prob;
}

__host__ __device__ double Gaussian(double mu, double sigma, double x) {
  return exp(-((mu - x) * (mu - x)) / (sigma * sigma) / 2.0) /
      sqrt(2.0 * M_PI * (sigma * sigma));
}

template<typename T/*, typename = std::enable_if_t<std::is_integral_v<T>>*/>
string to_string_with_zero_padding(const T &value, size_t total_length) {
  auto str = to_string(value);
  if (str.length() < total_length)
    str.insert(str.front() == '-' ? 1 : 0, total_length - str.length(), '0');
  return str;
}

#if defined(IMAGES)
__host__ void plot_particles(Robot r, const vector<Robot> &p, int itr) {
  Mat img = Mat::ones(world_size, world_size, CV_8UC3);

//  plot_particles particle positions
  for (auto &i: p) {
    img.at<Vec3b>(round(i.get_x()), round(i.get_y())) = Vec3b(0, 255, 0);
  }

  //  plot_particles position of robot
  img.at<Vec3b>(round(r.get_x()), round(r.get_y())) = Vec3b(0, 0, 255);


//  plot_particles landmark position
  for (auto &i: landmarks) {
    img.at<Vec3b>(round(i[0]), round(i[1])) = Vec3b(255, 255, 255);
  }

//  plot_particles wall
  for (int i = 0; i < world_size; ++i) {
    img.at<Vec3b>(i, 0) = Vec3b(255, 255, 255);
    img.at<Vec3b>(0, i) = Vec3b(255, 255, 255);
    img.at<Vec3b>(i, world_size - 1) = Vec3b(255, 255, 255);
    img.at<Vec3b>(world_size - 1, i) = Vec3b(255, 255, 255);

  }

  imwrite("images/" + to_string_with_zero_padding(itr, 3) + ".png", img);

}
#endif

__host__ double eval(Robot r, vector<Robot> p, int itr) {
  double sum = 0.0;
  for (int i = 0; i < p.size(); i++) {
    double dx = (p[i].get_x() - r.get_x() + fmod(world_size / 2.0, world_size) -
        (world_size / 2.0));
    double dy = (p[i].get_y() - r.get_y() + fmod(world_size / 2.0, world_size) -
        (world_size / 2.0));
    double err = sqrt(dx * dx + dy * dy);
    sum += err;
  }

#if defined(IMAGES)
  plot_particles(r, p, itr);
#endif

  return sum / (double) p.size();
}

__host__ __device__ double Robot::get_x() const { return x; }
__host__ __device__ void Robot::set_x(double x) { Robot::x = x; }
__host__ __device__ double Robot::get_y() const { return y; }
__host__ __device__ void Robot::set_y(double y) { Robot::y = y; }
__host__ __device__ double Robot::get_orientation() const {
  return orientation;
}
__host__ __device__ void Robot::set_orientation(double orientation) {
  Robot::orientation = orientation;
}
__host__ __device__ double Robot::get_forward_noise() const {
  return forward_noise;
}
__host__ __device__ void Robot::set_forward_noise(double forward_noise) {
  Robot::forward_noise = forward_noise;
}
__host__ __device__ double Robot::get_turn_noise() const { return turn_noise; }
__host__ __device__ void Robot::set_turn_noise(double turn_noise) {
  Robot::turn_noise = turn_noise;
}
__host__ __device__ double Robot::get_sense_noise() const {
  return sense_noise;
}

__host__ __device__ void Robot::set_sense_noise(double sense_noise) {
  Robot::sense_noise = sense_noise;
}
