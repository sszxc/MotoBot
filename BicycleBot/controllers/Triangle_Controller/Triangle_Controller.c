#include <webots/robot.h>
#include <webots/motor.h>
#define TIME_STEP 64

WbDeviceTag Steer_Motor;

#define PI 3.1416
#define STEER_MOTOR_MAX_SPEED  15.0
#define MAX_ANGLE  PI/4
#define MIN_ANGLE  -PI/4

void Turn_to(double position)
{
  wb_motor_set_velocity(Steer_Motor, STEER_MOTOR_MAX_SPEED);
  wb_motor_set_position(Steer_Motor, position);
}

int main(int argc, char **argv) {
  wb_robot_init();
  Steer_Motor = wb_robot_get_device("steer");
  wb_motor_set_position(Steer_Motor, 0.0);
  wb_motor_set_velocity(Steer_Motor, 0.0);
  
  int count = 10;
  int direction = 1;
  while (wb_robot_step(TIME_STEP) != -1) {
    if(direction == 1)
    {
      Turn_to(MAX_ANGLE);
      if(--count == 0)
      {
        direction = 0;
      }
    }
    else
    {
      Turn_to(MIN_ANGLE);
      if(++count == 10)
      {
        direction = 1;
      }
    }
  }
  wb_robot_cleanup();
  return 0;
}
