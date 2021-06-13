/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>


using std::string;
using std::vector;
using std::normal_distribution;
// using std::discrete_distribution;

// using std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  std::default_random_engine gen;
  num_particles = 100;  // TODO: Set the number of particles
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);
  
  for(int i=0;i<num_particles;++i){
    
    struct Particle val;
    val.x=dist_x(gen);
    val.y=dist_y(gen);
    val.theta=dist_theta(gen);
    val.weight=1;
    particles.push_back(val);
  
  }
  is_initialized=true;

  

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  double yaw_angle=yaw_rate*delta_t;
  for(int i=0;i<particles.size();++i){
    
  	if (fabs(yaw_rate) < 0.00001){
    	particles[i].x=particles[i].x + velocity*delta_t*cos(particles[i].theta);
    	particles[i].y=particles[i].y + velocity*delta_t*sin(particles[i].theta);

    } 
    else{
   	 	particles[i].x=particles[i].x + velocity/yaw_rate*(sin(particles[i].theta + yaw_angle)-sin(particles[i].theta));
    	particles[i].y=particles[i].y + velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_angle));
    	particles[i].theta=particles[i].theta+yaw_rate*delta_t;
    }
  	normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
  	normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
  	normal_distribution<double> dist_theta(particles[i].theta,std_pos[2]);
    
    particles[i].x=dist_x(gen);
    particles[i].y=dist_y(gen);
    particles[i].theta=dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
//   	float diff=std::numeric_limits<float>::max();
// 	for(int i=0;i<observations.size();++i){
//       for(int j=0;j<predicted.size();++j){
        
//          float euclid_dist=dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
//          if(diff>euclid_dist){
// 			diff=euclid_dist;
//            	observations[i].id=predicted[j].id;
//          }
      
//       }
    
//     }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */


  for(int i=0; i<particles.size(); ++i){

    particles[i].weight=1;
    double final_weight=1;
    for(int j=0; j<observations.size(); ++j){
    	LandmarkObs obs_map;
    	double diff=std::numeric_limits<float>::max();
    	double mu_x,mu_y;
   		double obs_weight;
    //   Transforming observations to map coordinates
    	obs_map.x=particles[i].x + (cos(particles[i].theta)*observations[j].x)-(sin(particles[i].theta)*observations[j].y);
      	obs_map.y=particles[i].y + (sin(particles[i].theta)*observations[j].x)+(cos(particles[i].theta)*observations[j].y);
      	

      
	// 	Associating best landmark to the current observation  
      	
        for(int k=0; k < map_landmarks.landmark_list.size(); ++k){
        
         if(fabs(map_landmarks.landmark_list[k].x_f - particles[i].x) <= sensor_range && fabs(map_landmarks.landmark_list[k].y_f - particles[i].y) <= sensor_range) { 
         	float euclid_dist=dist(obs_map.x,obs_map.y,map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f);
            if(diff>euclid_dist){
				diff=euclid_dist;
              	
           		obs_map.id=map_landmarks.landmark_list[k].id_i;
           		mu_x=map_landmarks.landmark_list[k].x_f;
        		mu_y=map_landmarks.landmark_list[k].y_f;
         }  
         } 
      
     }
      
//      Calculating the observation weight using multi-variate Gaussian distribution
     obs_weight=multiv_prob(std_landmark[0], std_landmark[1],obs_map.x, obs_map.y, mu_x, mu_y);

     final_weight*=obs_weight; 

    }
   //Updating the particles_weight    
   particles[i].weight=final_weight;
   

     
 }
  
 //Summing the weights 
 double sum=0.0;
 for(int i=0;i<particles.size();++i){
 	
   sum=sum+particles[i].weight;
 
 }  
  
 //Normalizing the weights
 for(int i=0;i<particles.size();++i){
 	
   particles[i].weight=particles[i].weight/sum;
 
 } 
   

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;

  double beta=0.0f;
  double mw=0;
  vector<Particle> new_particles;
//   Finding max weight
  for(int i=0;i<particles.size();++i){
  		mw=std::max(mw,particles[i].weight);
  }
	
  //Used uniform real distribution for randomisation
  std::uniform_real_distribution<> dist_index(0,num_particles-1);
  int index=dist_index(gen);
  for(int i=0;i<particles.size();++i){
    std::uniform_real_distribution<> distr(0.0,2.0*mw);
    beta+=distr(gen);
    while(beta>particles[index].weight){
      
    	beta-=particles[index].weight;
        index=(index+1)% num_particles;

    }  
    new_particles.push_back(particles[index]);
 
  }
  particles=new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}