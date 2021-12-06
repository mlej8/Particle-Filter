# Particle Filter

## To Run
1. `cmake . && make`
3. Run either `particle_filer_gpu` or `particle_filter_cpu`
   1. ```
      ./particle_filter_gpu <optional:number_iteration(default:1000)> <optional:number_particles (default:10)> <number of threads per block (default:64)>
      
   2. ```
      ./particle_filter_cpu <optional:number_iteration(default:1000)> <optional:number_particles (default:10)>

## Generate image for each loop
Make sure you have OpenCV installed. See [here](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) for install guide.
1. Compile with `cmake . -D IMAGES=1 && make`
2. Run the program. (Max of 70k particles)
3. The images will be in `images/`.
4. To generate a gif, run `convert -delay 50 -loop 0 images/*.png myimage.gif`.
   1. If you do not have imagemagick, run `sudo apt-get install imagemagick`
5. Make sure to clear the `images/` folder if you are running it another time

## Notes
* The sequential implementation needs to end with `.cu` in order to let the CUDA compiler link the `robot.cuh` library. 
It is done this way to reduce duplicated code. No GPU kernels are used.