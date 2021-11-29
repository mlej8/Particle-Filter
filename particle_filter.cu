#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/host_vector.h>
#include <thrust/scan.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "robot.cuh"

using namespace std;

// TODO fix bug where if block_size > 512 code fails

__global__ void weight_normalization(double *weights, double weight_sum, const int N){
  int index = threadIdx.x + blockDim.x * blockIdx.x;
  if (index < N) {
    weights[index] /= weight_sum;
  }
}

/**
 * Simulates robot motion for each particle, performs measurement update and
 * assigns a weight to each particle.
 */
__global__ void particle_filter(Robot *particles, double *weights,
                                const double theta, const double distance,
                                const int N, const double *Z,
                                const int num_landmarks,
                                const double *landmarks_gpu) {
  int index = threadIdx.x + blockDim.x * blockIdx.x;
  if (index < N) {
    particles[index].move(theta, distance);

    // likelihood of the real observation based on the current particle's state
    double prob = 1.0;
    for (int i = 0; i < num_landmarks; i++) {
      double dist =
          sqrt((particles[index].get_x() - landmarks_gpu[i * 2 + 0]) *
                   (particles[index].get_x() - landmarks_gpu[i * 2 + 0]) +
               (particles[index].get_y() - landmarks_gpu[i * 2 + 1]) *
                   (particles[index].get_y() - landmarks_gpu[i * 2 + 1]));
      prob *= Gaussian(dist, particles[index].get_sense_noise(), Z[i]);
    }
    weights[index] = prob;
  }
}
int main(int argc, char *argv[]) {
  // physical robot (ground truth)
  auto start = chrono::high_resolution_clock::now();

  if (argc != 4) {
    cout << "Usage: ./particle_filter_gpu <number of particles> <number of "
            "iterations of particle filtering> <number of threads per block>"
         << endl;
    exit(1);
  }

  // physical robot (ground truth)
  Robot my_robot;

  // number of particles (TODO: set default to 1000)
  int N = atoi(argv[1]);

  // number of particle filter iterations (TODO: set default to 10)
  int T = atoi(argv[2]);

  int block_size = atoi(argv[3]);
  size_t num_block = (N + block_size - 1) / block_size;
  size_t particles_size = N * sizeof(Robot);
  int num_landmarks = sizeof(landmarks) / sizeof(landmarks[0]);
  int landmark_dim = sizeof(landmarks[0]) / sizeof(double);
  size_t landmark_size = sizeof(double) * num_landmarks * landmark_dim;

  // initialize N random particles (robots)
  // list of particles (guesses as to where the robot might be - each particle
  // is a vector representing the state of the robot (x,y,theta) theta is the
  // angle relative to the x-axis)
  vector<Robot> particles(N);
  Robot *particles_gpu;
  thrust::device_vector<double> weights_gpu(N);
  thrust::host_vector<double> weights(N);
  double *landmarks_gpu;
  cudaMalloc(&landmarks_gpu, landmark_size);
  cudaMalloc(&particles_gpu, particles_size);
  
  cudaMemcpy(landmarks_gpu, landmarks, landmark_size, cudaMemcpyHostToDevice);
  cudaMemcpy(particles_gpu, particles.data(), particles_size,
             cudaMemcpyHostToDevice);

  for (int j = 0; j < T; j++) {
    double theta = uniform_distribution_sample() * M_PI / 2;
    double distance = (uniform_distribution_sample() * 9.0) + 1;
    my_robot.move(theta, distance);

    // detect distance of the real robot to the landmarks in our world (returns
    // list of distance to each obstacle)
    vector<double> Z = my_robot.sense();
    thrust::device_vector<double> Z_gpu(Z);

    particle_filter<<<num_block, block_size>>>(
        particles_gpu, thrust::raw_pointer_cast(weights_gpu.data()), theta,
        distance, N, thrust::raw_pointer_cast(Z_gpu.data()), Z.size(),
        landmarks_gpu);
    cudaDeviceSynchronize();

    // weights summation via parallelization
    double weights_sum = thrust::reduce(weights_gpu.begin(), weights_gpu.end()); // , thrust::plus<double>()

    weight_normalization<<<num_block, block_size>>>(thrust::raw_pointer_cast(weights_gpu.data()), weights_sum, N);
    cudaDeviceSynchronize();

    // copying normalized particles' weights from GPU to CPU
    thrust::copy(weights_gpu.begin(), weights_gpu.end(), weights.begin());

    // copying updated particles to CPU
    cudaMemcpy(particles.data(), particles_gpu, particles_size,
               cudaMemcpyDeviceToHost);

    // compute cumulative distribution function (CDF)
    thrust::device_vector<double> cdf_gpu(weights_gpu.size());
    thrust::inclusive_scan(weights_gpu.begin(), weights_gpu.end(), cdf_gpu.begin());
    thrust::host_vector<double> cdf(cdf_gpu);

    int k = 0;
    auto u = [&N](int n) {
      return (((n - 1) + uniform_distribution_sample()) / N);
    };

    // systematic resampling of new particles
    vector<Robot> new_particles;
    for (int i = 1; i <= N; i++) {
      while (cdf[k] < u(i)) {
        k += 1;
      }
      new_particles.push_back(particles[k]);
    }
    particles = new_particles;

    // copy new particles to GPU for next iteration
    cudaMemcpy(particles_gpu, particles.data(), particles_size,
               cudaMemcpyHostToDevice);

    cout << eval(my_robot, particles) << endl;
  }

  cudaFree(particles_gpu);
  cudaFree(landmarks_gpu);
  auto finish = chrono::high_resolution_clock::now();
  std::cout
      << block_size << "," << N << ","
      << chrono::duration_cast<chrono::nanoseconds>(finish - start).count()
      << "\n";

  return 0;
}