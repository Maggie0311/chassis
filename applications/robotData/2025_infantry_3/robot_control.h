#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "controlMsg.h"
#include "robot_data.h"

int chassis_motor_init();
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw);

int gim_motor_init();

#endif