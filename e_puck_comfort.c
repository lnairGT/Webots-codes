//Attachment behavior - based on work by Maxim Likhachev and Ronald Arkin
//Action-selection behavior coordination of attachment and ingestive 
//behaviors

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include <webots/pen.h>
#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <webots/gps.h>

#define SPEED 200
#define TIMESTEP 64
#define DIST_TOL 0.2 
#define ANGLE_TOL 0.1
#define N_STEPS 700

typedef struct Vector{
  float u;
  float v;
} Vector; 

//Functions for different vector operations
void minus(Vector *v, Vector *v1, Vector *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

void sum(Vector *v, Vector *v1, Vector *v2) {
  v->u = v1->u + v2->u;
  v->v = v1->v + v2->v;
}

float norm(Vector *v) {
  return sqrt(v->u*v->u + v->v*v->v);
}

void normalize(Vector *v) {
  double n = norm(v);
  v->u /= n;
  v->v /= n;
}

double angle(Vector *v1, Vector *v2) {
  double theta = atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
  if (theta < 0)
    theta += 2*M_PI;
    
  return (theta);
}

void scale(double s, Vector *v1) {
  v1->u *= s;
  v1->v *= s;
}

int main(){

    //Initializing compass and gps devices
    WbDeviceTag compass, gps, pen, ps[8];  
    wb_robot_init();
    
    //Parameters for the wander behavior  
    int i;
    int steps = N_STEPS;
    int noise_u, noise_v;
    double wander_gain = 0.9;
    
    FILE *f1 = fopen("Dist_total.csv", "w+");
    
    //Parameters for the attachment behavior
    double C = 0.0;
    double A = 1;
    double Ah = 1;
    double Al = 0.1;
    
    double Ch = -0.6;
    double Cl = 0.6;
    
    double phi;
    double D;
    
    double N = 10;
    double alpha = 1;
    double ds = 0.5;
    double dz = 0.5;
    float obj_dist;
    
    //Ingestive behavior parameters //CHANGE TO INTERNAL ENERGY
    double hunger = 0;
    double hunger_incr = 0.05;
    double hunger_threshold = 7;
    double h_max = 10;

    //Vector location  
    Vector target_loc = {-0.2, -0.3};

    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass,TIMESTEP);
    
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps,TIMESTEP);

    char ps_names[8][4] = {"ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"};
    double ps_values[8];
    double dist_th = 150;
    
    pen = wb_robot_get_device("pen");
    wb_pen_set_ink_color(pen, 0, 0.5);

    for (i=0; i<8; i++){
      ps[i] = wb_robot_get_device(ps_names[i]);
      wb_distance_sensor_enable(ps[i], TIMESTEP);
    }

    while (wb_robot_step(TIMESTEP) != -1){
    
      wb_pen_write(pen, 1); 
      
      //Initializing speeds
      double left_speed = SPEED;
      double right_speed = SPEED;
      
      //Retrieve compass and gps values
      const double *pos3D = wb_gps_get_values(gps);
      const double *north3D = wb_compass_get_values(compass);
      
      if (hunger < h_max)
        hunger = hunger + hunger_incr;
      
      for (i=0; i<8; i++){
        ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      }
      
      //Current position
      Vector pos = {pos3D[0], pos3D[2]};
      //Current heading direction 
      double rad = atan2(north3D[0], north3D[2]) - 1.5708;
      
      if (rad < 0.0)
        rad = rad + 2*M_PI;
     
      Vector heading = {sin(rad), -cos(rad)}; 
       
      //Compute vector from current position pointing to target location 
      Vector target_dir;
      minus(&target_dir, &target_loc, &pos);
      
      //Compute object distance
      obj_dist = norm(&target_dir);
      normalize(&target_dir);
      
      if (C > Cl)
        phi = Al;
      else if (C >= Ch && C <= Cl)
        phi = ((Ah-Al)/(Ch-Cl))*C - ((Ah-Al)/(Ch-Cl))*Ch + Ah;
      else
        phi = Ah;
      
      if (obj_dist <= ds)
        D = 0;
      else if (obj_dist > ds && obj_dist < ds+dz)
        D = (1/dz)*obj_dist - (ds/dz);
      else
        D = 1;
    
      A = N*phi*alpha*D;
      scale(A, &target_dir);
      
      printf("Val of A is %f \n", A);
      
      Vector vec_sum;
      
      if (steps == N_STEPS){
        noise_u = rand() % 100 - 50;
        noise_v = rand() % 100 - 50;
        Vector random = {noise_u, noise_v};
        normalize(&random);
        scale(wander_gain, &random);
        sum(&vec_sum, &target_dir, &random);
        fprintf(f1, "%f,", obj_dist);
        steps = steps - 1;
      }
      else if (steps <= 0){
        steps = N_STEPS;
      }
      else{
        steps = steps - 1;
      }
      
      //INGESTIVE BEHAVIOR - go straight up when hungry
      if (hunger > hunger_threshold){
        vec_sum.u = 0.0;
        vec_sum.v = 1.0;
      }
      
      //Compute angle between target direction and current heading direction
      double beta = angle(&heading, &vec_sum);
      printf("%d %d noise is \n", noise_u, noise_v);
      printf("%f %f heading \n", heading.u, heading.v);
      printf("%f %f vec_sum \n", vec_sum.u, vec_sum.v);
      printf("%f beta \n", beta);
      
      //Turn by the computed angle
      if (beta < -ANGLE_TOL){
        left_speed = SPEED;
        right_speed = -SPEED;
      }
      else if (beta > ANGLE_TOL){
        left_speed = -SPEED;
        right_speed = SPEED;
      }
      else{
        left_speed = SPEED - M_PI + beta;
        right_speed = SPEED - M_PI - beta;
      }
      
      bool left_obstacle = ps_values[0] > dist_th || ps_values[1] > dist_th || ps_values[2] > dist_th;
      bool right_obstacle = ps_values[5] > dist_th || ps_values[6] > dist_th || ps_values[7] > dist_th;
    
      //Turn in the direction of the new computed vector
      if (right_obstacle){
        left_speed = SPEED;
        right_speed = SPEED/2;
      }
      else if (left_obstacle){
        left_speed = SPEED/2;
        right_speed = SPEED;
      }

      wb_differential_wheels_set_speed(left_speed, right_speed);
    
    }
  wb_robot_cleanup();
  return 0;
} 