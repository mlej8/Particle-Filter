#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
using namespace std;

double landmarks[4][2] = {{20.0, 20.0}, {80.0, 80.0}, {20.0, 80.0}, {80.0, 20.0}};
double world_size = 100;

//taken from https://www.tutorialspoint.com/generate-random-numbers-following-a-normal-distribution-in-c-cplusplus
double rand_gen() {
   // return a uniformly distributed random value
   return ( (double)(rand()) + 1. )/( (double)(RAND_MAX) + 1. );
}
double normalRandom() {
   // return a normally distributed random value
   double v1=rand_gen();
   double v2=rand_gen();
   return cos(2*3.14*v2)*sqrt(-2.*log(v1));
}



class Robot {
    public:
        double x;
        double y;
        double orientation;
        double forward_noise;
        double turn_noise;
        double sense_noise;
        Robot(){
            x = ((double) rand()/RAND_MAX)*world_size;
            y = ((double) rand()/RAND_MAX)*world_size;
            orientation = ((double) rand()/RAND_MAX)*2.0*M_PI;
            forward_noise = 0.0;
            turn_noise = 0.0;
            sense_noise = 0.0;
        }
        void set(double new_x, double new_y, double new_orientation);
        void set_noise(double f_noise, double t_noise, double s_noise);
        std::vector<double> sense();
        Robot move(double turn, double forward);
        double measurement_prob(std::vector<double> measurement); 
        double eval(Robot r, std::vector<Robot> p, int counter);

};

void Robot::set(double new_x, double new_y, double new_orientation){
    if (new_x <0 || new_x >=world_size){
        throw std::invalid_argument("X coordinate out of bound");
    }
    if (new_y <0 || new_y >= world_size){
        throw std::invalid_argument("Y coordinate out of bound");
    }
    if (new_orientation < 0 || new_orientation >= 2*M_PI){
        throw std::invalid_argument("Orientation must be in [0..2pi]");
    }
    x = new_x;
    y = new_y;
    orientation = new_orientation;
}

void Robot::set_noise(double f_noise, double t_noise, double s_noise){
    forward_noise = f_noise;
    turn_noise = t_noise;
    sense_noise = s_noise;
}

std::vector<double> Robot::sense(){
    std::vector<double> Z;
    for(int i=0;i<(sizeof landmarks/ sizeof landmarks[0]); i++){
        double dist = sqrt((x-landmarks[i][0])*(x-landmarks[i][0]) + (y-landmarks[i][1])*(y-landmarks[i][1]));
        dist += normalRandom()*forward_noise;
        Z.push_back(dist);
    }
    return Z;
}

Robot Robot::move(double turn, double forward){
    if (forward < 0 ){
        throw std::invalid_argument("Robot cant move backwards");
    }
    double new_orientation = orientation + turn + normalRandom()*turn_noise;
    new_orientation = fmod(new_orientation, 2* M_PI);
    double dist = forward + normalRandom()*forward_noise;
    double new_x = x+ cos(orientation)*dist;
    double new_y = y+ sin(orientation)*dist;
    new_x = fmod(new_x, world_size);
    new_y = fmod(new_y, world_size);

    Robot res;
    res.set(new_x, new_y, new_orientation);
    res.set_noise(forward_noise, turn_noise, sense_noise);
    return res;
}


double Gaussian(double mu, double sigma, double x){
    return exp(- ((mu - x)*(mu-x)) / (sigma*sigma) / 2.0) / sqrt(2.0 * M_PI * (sigma*sigma));
}

double Robot::measurement_prob(std::vector<double> measurement){
    double prob = 1.0;
    for (int i=0;i<(sizeof landmarks/ sizeof landmarks[0]);i++){
        double dist = sqrt((x-landmarks[i][0])*(x-landmarks[i][0]) + (y-landmarks[i][1])*(y-landmarks[i][1]));
        prob *= Gaussian(dist, sense_noise, measurement[i]);
    }
    return prob;
}

double eval(Robot r, std::vector<Robot> p){
    double sum = 0.0;
    for (int i=0;i<p.size();i++){
        double dx = (p[i].x-r.x + fmod(world_size/2.0, world_size) - (world_size/2.0));
        double dy = (p[i].y - r.y + fmod(world_size/2.0, world_size) - (world_size/2.0));
        double err = sqrt(dx * dx + dy * dy);
        sum += err;
    }
    return sum/(double)p.size();
}

int main(){
    Robot myrobot;
    myrobot = myrobot.move(0.1, 5.0);
    std::vector<double> Z = myrobot.sense();
    int N = 1000;
    int T = 10;

    std::vector<Robot> p;
    for (int i=0;i<N;i++){
        Robot r;
        r.set_noise(0.05, 0.05, 5.0);
        p.push_back(r);
    }

    for(int j=0;j<T;j++){
        myrobot = myrobot.move(0.1, 5.0);
        Z = myrobot.sense();

        std::vector<Robot> p2;
        for (int k=0;k<N;k++){
            p2.push_back(p[k].move(0.1, 5.0));
        }
        p = p2;

        std::vector<double> w;
        for (int k=0;k<N;k++){
            w.push_back(p[k].measurement_prob(Z));
        }

        std::vector<Robot> p3;
        int index = (int)((rand()/RAND_MAX)*N);
        double beta = 0.0;
        double max_w = w[0];
        for(int l=0;l<w.size();l++){
            if(w[l]>max_w){
                max_w = w[l];
            }
        }

        for (int m=0;m<N;m++){
            beta += (rand()/RAND_MAX)*2.0*max_w;
            while (beta > w[index]){
                beta -= w[index];
                index = (index+1)%N;
            }
            p3.push_back(p[index]);
        }
        p=p3;

        cout << eval(myrobot, p) << endl;

    }

    return 0;
}