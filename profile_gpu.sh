#!/usr/bin/bash

mkdir -p tmp
mkdir -p reports
for run in {1..5} ; do
  for num_particles in "100" "500" "1000" "5000" "10000" "50000" "100000" "500000" "1000000"
    do
      for block_size in "1" "2" "4" "8" "16" "32" "64" "128" "256" "512"
        do
#          f_name="$block_size"_"$num_particles"_run_"$run"
#          nsys profile --output tmp/"$f_name" ./particle_filter_gpu $num_particles 15 $block_size
#          nsys stats tmp/"$f_name".qdrep  >> reports/"$f_name".txt
          ./particle_filter_gpu $num_particles 15 $block_size >> Profiling/parallel_manual.csv
        done
    done
done