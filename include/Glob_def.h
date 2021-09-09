#ifndef Glob_def_H
#define Glob_def_H

typedef enum        COMPO_IDX {X = 0, Y, Z} Compo_Index;

// Constant
#define DEF_PI          3.14159265358979323846
#define DEF_GRAVITY     9.80665 /* unit - m/s^2*/

// Math
#define DEF_D2R(a)   ((a) * (DEF_PI/180.0))
#define DEF_R2D(a)   ((a) * (180.0/DEF_PI))
#define DEF_ABS(a)         (((a) < 0) ? (-(a)) : (a))
#define DEF_MIN(a, b)      (((a) < (b)) ? (a) : (b))
#define DEF_MAX(a, b)      (((a) > (b)) ? (a) : (b))
#define DEF_SIGN(a)        (((a) < 0) ? (-1) : (1))
#define DEF_constrain(a,min,max)   (a < min) ? min : ((a > max) ? max : a)

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <float.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Bool.h>

#include "Function.h"

using namespace std;

#endif
