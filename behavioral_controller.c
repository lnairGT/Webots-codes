/* A simple behavioral controller with four behaviors - MOVE_TO_FOOD, 
FIGHT_CONSPECIFIC, RETREAT, AVOID_OBSTACLE
And two internal variables - hunger, fear 
The chosen behavior depends on the variable with higher value - Action
Selection behavior coordination strategy
*/

//AUTHOR: LAKSHMI NAIR

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/pen.h>
#include <webots/distance_sensor.h>

#define SPEED 200
#define TIME_STEP 64
#define SPEED_UNIT 0.00628

enum BLOB_TYPE {RED, GREEN, BLUE, NONE};

static double left_speed, right_speed;

//BEHAVIOR DEFINITIONS: Chosen behavior dependent on internal variable with highest value
void MOVE_TO_FOOD(double hunger, double fear){
  if (hunger >= fear){
    printf("Move to tree \n");
    left_speed = SPEED;
    right_speed = SPEED;
  }
  else{
    RETREAT();
  }  
}

void FIGHT_CONSPECIFIC(double hunger, double fear){
  if (fear > hunger){
    printf("Fight conspecific \n");
    left_speed = SPEED;
    right_speed = SPEED;
  }
  else{
    printf("Retreat \n");
    RETREAT();
  }
}

void RETREAT(void){
  left_speed = SPEED;
  right_speed = SPEED/2;
}

void AVOID_OBSTACLE(bool left_obstacle, bool right_obstacle){
  if (right_obstacle){
    printf("Right obstacle detected \n");
    left_speed = SPEED; 
    right_speed = -SPEED;
  }
  else if (left_obstacle){
    printf("Left obstacle detected \n");
    left_speed = -SPEED; 
    right_speed = SPEED;
  }
}

int main() {

  //Logging the internal variables
  FILE *f1 = fopen("Hunger_level.csv","w+");
  FILE *f2 = fopen("Fear_level.csv","w+");

  //Devices to be initialized
  WbDeviceTag camera, pen, ps[8];

  //INTERNAL VARIABLES
  float hunger = 0.0;
  float fear = 0.0;
  float hunger_max = 5.0; //Max value for hunger variable
  float hunger_incr = 0.05; //Hunger increment value over time

  //OTHER VARIABLES FOR IMAGE PROCESSING
  int width, height;
  int pause_counter=0;
  int i, j;
  int red, blue, green;
  const unsigned char *image;
  enum BLOB_TYPE current_blob = NONE;
  
  //VARIABLES ASSOCIATED WITH DISTANCE SENSOR
  double ps_values[8];
  int dist_th = 150;
  int flag = 0;
  int turn_counter = 0;
  float max_ps;

  wb_robot_init();
  
  //Initializing camera
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  
  //Initializing pen
  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0, 0.5);
  
  //Initializing distance sensors
  char ps_names[8][4] = {"ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"};
  
  for (i=0; i<8; i++){
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  //BEGIN ROBOT ACTION LOOP
  while (wb_robot_step(TIME_STEP) != -1){
    
    //Enable pen and Distance sensors
    wb_pen_write(pen,1);
  
    for (i = 0; i < 8; i++){
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    }
    
    //PRIOR TO BEHAVIOR SELECTION
    if (flag == 0){  
      
      //Increment hunger over time
      if (hunger < hunger_max)
        hunger += hunger_incr;
      
      //Capture image from camera
      image = wb_camera_get_image(camera);
      
      red = 0;
      blue = 0;
      green = 0;
      
      //Analyze camera image
      for (i = (width/3)-5; i < (2*width/3)+5; i++) {
          for (j = (height/2)-5; j < (3*height/4)+5; j++) {
            red += wb_camera_image_get_red(image, width, i, j);
            blue += wb_camera_image_get_blue(image, width, i, j);
            green += wb_camera_image_get_green(image, width, i, j);
          }
        }  
      
      //Check color of object  
      if (pause_counter < 10){
          if ((red > 3*green) && (red > 3*blue))
            current_blob = RED;                     //Strong conspecific
          else if ((green > 3*red) && (green > 3*blue))
            current_blob = GREEN;                   //Food source
          else if ((blue > 3*red) && (blue > 3*green))
            current_blob = BLUE;                    //Weak conspecific
          else
            current_blob = NONE;
        }
        else
          current_blob = NONE;
      
      //BEHAVIORAL SELECTION PHASE DEPENDING ON MAX INTERNAL VARIABLE
      if (current_blob == NONE){ 
          left_speed  = SPEED;
          right_speed =  SPEED;
          fear = 0;
      }
            
      //If food observed, call MOVE_TO_FOOD
      else if (current_blob == GREEN){
        printf("Green blob detected \n");
        MOVE_TO_FOOD(hunger, fear);
        fear = 0;
        flag = 1;
      }
      
      //If conspecific observed, call FIGHT_CONSPECIFIC
      else if (current_blob == RED){
        printf("Red Blob detected \n");
        fear = 4;
        FIGHT_CONSPECIFIC(hunger, fear);
        flag = 1;
      }
      else if (current_blob == BLUE){
        printf("Blue Blob detected \n");
        fear = 2;
        FIGHT_CONSPECIFIC(hunger, fear);
        flag = 1;
      }
    }
    
    //EXECUTE BEHAVIOR AFTER BEHAVIORAL SELECTION
    if (flag == 1){

      left_speed = left_speed;
      right_speed = right_speed;
      
      if (right_speed == SPEED/2){
        turn_counter += 1;
      }
      
      if (turn_counter >= 60){
        left_speed = SPEED;
        right_speed = SPEED;
        turn_counter = 0;
        flag = 0;
      }
    }

    //Checking for obstacles
    bool left_obstacle = ps_values[0] > dist_th || ps_values[1] > dist_th || ps_values[2] > dist_th;
    bool right_obstacle = ps_values[5] > dist_th || ps_values[6] > dist_th || ps_values[7] > dist_th;
    
    //OBSTACLE AVOIDANCE BEHAVIOR
    if (left_obstacle || right_obstacle){
      AVOID_OBSTACLE(left_obstacle, right_obstacle);
      flag = 0;
    }
    
    //FINALIZE SPEEDS, SAVE INTERNAL VARIABLES
    wb_differential_wheels_set_speed(left_speed, right_speed);
    fprintf(f1, "%f,", hunger);
    fprintf(f2, "%f,", fear);
  }
  
  wb_robot_cleanup();
  return 0;  
}
