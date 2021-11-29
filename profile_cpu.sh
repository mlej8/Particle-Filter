#!/usr/bin/bash

# max of 768
for run in {1..5} ; do
    for num_particles in "100" "500" "1000" "5000" "10000" "50000" "100000" "500000" "1000000"
      do
        echo "running_$run_$num_particles"
        ./particle_filter_cpu "$num_particles" 15 >> Profiling/sequential.csv
      done
done
