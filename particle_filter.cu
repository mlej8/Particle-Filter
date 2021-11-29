#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/host_vector.h>

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "robot.cuh"

using namespace std;

/**
 * Simulate robot motion for each particle and perform importance weight
 * computation
 */
__global__ void particle_filter(Robot *particles, double *weights,
                                const double theta, const double distance,
                                const int N, const double *Z_gpu,
                                const int num_landmarks,
                                const double *landmarks) {
  int index = threadIdx.x + blockDim.x * blockIdx.x;
  if (index < N) {
    printf("???");
    particles[index].move(theta, distance);
    double prob = 1.0;
    for (int i = 0; i < num_landmarks; i++) {
      double dist = sqrt((particles[index].get_x() - landmarks[i * 2 + 0]) *
          (particles[index].get_x() - landmarks[i * 2 + 0]) +
          (particles[index].get_y() - landmarks[i * 2 + 1]) *
              (particles[index].get_y() - landmarks[i * 2 + 1]));
      prob *= Gaussian(dist, particles[index].get_sense_noise(), Z_gpu[i]);
    }
    weights[index] = prob;
  }
}
int main(int argc, char *argv[]) {
  // physical robot (ground truth)
  Robot my_robot;

  if (argc != 4) {
    cout << "Usage: ./particle_filter_gpu <number of particles> <number of "
            "iterations of particle filtering> <number of threads per block>"
         << endl;
    exit(1);
  }

  // number of particles (TODO: set default to 1000)
  int N = atoi(argv[1]);

  // number of iterations of particle filter (TODO: set default to 10)
  int T = atoi(argv[2]);

  int block_size = atoi(argv[3]);
  size_t num_block = (N + block_size - 1) / block_size;
  size_t particles_size = N * sizeof(Robot);
  int num_landmarks = sizeof(landmarks) / sizeof(landmarks[0]);
  int landmark_dim = sizeof(landmarks[0]) / sizeof(double);
  size_t landmark_size = sizeof(double) * num_landmarks * landmark_dim;

  // initialize N random particles (robots)
  // list of particles  (guesses as to where the robot might be - each particle
  // is a vector representing the state of the robot (x,y,theta)  theta is the
  // angle relative to the x-axis)
  vector<Robot> particles(N);
  Robot *particles_gpu;
  thrust::device_vector<double> weights_gpu(N);
  thrust::host_vector<double> weights(N);
  double *landmarks_gpu;
  cudaMalloc(&landmarks_gpu, landmark_size);
  cudaMemcpy(landmarks_gpu, landmarks, landmark_size, cudaMemcpyHostToDevice);

  // copy those particles on the GPU
  cudaMalloc(&particles_gpu, particles_size);
  // cudaMallocManaged(&weights, N * sizeof(double));
  cudaMemcpy(particles_gpu, particles.data(), particles_size,
             cudaMemcpyHostToDevice);

  for (int j = 0; j < T; j++) {
    double theta = uniform_distribution_sample() * M_PI / 2;
    double distance = (uniform_distribution_sample() * 9.0) + 1;
    my_robot.move(theta, distance);
    vector<double> Z = my_robot.sense();
    thrust::device_vector<double> Z_gpu(Z);

    particle_filter<<<num_block, block_size>>>(
        particles_gpu, thrust::raw_pointer_cast(weights_gpu.data()), theta,
        distance, N, thrust::raw_pointer_cast(Z_gpu.data()), Z.size(),
        landmarks_gpu);
    cudaDeviceSynchronize();

    thrust::copy(weights_gpu.begin(), weights_gpu.end(), weights.begin());
    double max_w = *(thrust::max_element(weights.begin(), weights.end()));

    cudaMemcpy(particles.data(), particles_gpu, particles_size,
               cudaMemcpyDeviceToHost);

    // resampling
    vector<Robot> new_particles;
    int index = (int) (uniform_distribution_sample() * N);
    double beta = 0.0;
    for (int m = 0; m < N; m++) {
      beta += uniform_distribution_sample() * 2.0 * max_w;
      while (beta > weights[index]) {
        beta -= weights[index];
        index = (index + 1) % N;
      }
      new_particles.push_back(particles[index]);
    }
    particles = new_particles;
    cudaMemcpy(particles_gpu, particles.data(), particles_size,
               cudaMemcpyHostToDevice);

    cout << eval(my_robot, particles) << endl;
  }

  cudaFree(particles_gpu);
  return 0;
}