/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <Glob_def.h>
#include "rohang2/GpsPoint.h"
#include "rohang2/CameraPoint.h"
/*---------------------------------------------------------------
^
| : x axis


<<-- : y axis

---------------------------------------------------------------*/

ros::Publisher set_pos_pub;
ros::Publisher ardu_pub;
std_msgs::Bool arduino_state;

float height = 30.0f;
float r_speed = 3.0f;
float h_mis = 1.0f;
float r_no = 20.0f + 5.0f; // Radius of No Drone Area [m]
float r_area = 0.5f;
std::vector<float> Correct {0,0,0}; // Correct Information [Px, Py, yaw]
bool f_mis = false;
int Pos_Mode = 0; // 0:ENU, 1:LLH, 2:ECEF
int arduPubCount = 0;
int CamCount = 0;
float alpha = 35.0;

// LLH ( Lat, Long, Alt )
std::vector<double> L_home {37.47007, 127.30317, 0}; // Home Position
std::vector<double> L_no {37.46975, 127.30310, 0};   // Center of No Drone Area
std::vector<double> L_mis {37.46943, 127.30305, 0};  // Mission Position
std::vector<double> L_ret {37.46920, 127.30301, 0};  // return Position

// ECEF ( X, Y, Z)
std::vector<double> E_home {-3071630.58,4031628.74,3858928.18}; // Home Position
std::vector<double> E_no {-3071638.75,4031649.68,3858899.99};   // Center of No Drone Area
std::vector<double> E_mis {-3071648.33,4031669.55,3858871.80};  // Mission Position
std::vector<double> E_ret {-3071654.92,4031684.04,3858851.54};  // return Position
//*/

// EN ( East, North)
std::vector<float> P_home {0, 0}; // Home Position
std::vector<float> P_no {-10, -50};   // Center of No Drone Area
std::vector<float> P_mis {0, -100};  // Mission Position
std::vector<float> P_ret {100, -10};  // return Position

// Extend No Drone Zone
bool f_NoEx = false;
std::vector<double> L_ex {0,0,0}; // Center of No Drone Area(extra1)
std::vector<double> E_ex {0,0,0}; // Center of No Drone Area(extra1)
std::vector<float>  P_ex {50, -100}; // Center of No Drone Area(extra1)

void arduino_cb(const std_msgs::Bool::ConstPtr& msg){
    f_mis=(*msg).data;
}

rohang2::GpsPoint gps;
rohang2::CameraPoint camera;
void cameraCallback(const rohang2::CameraPoint::ConstPtr& msg){
	camera = *msg;
	std::cout << camera.a << ", " << camera.b << std::endl;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pose;
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pose = *msg;
}

sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    global_pose = *msg;
}

void set_setpoint(geometry_msgs::PoseStamped way, geometry_msgs::PoseStamped my)
{
    geometry_msgs::PoseStamped set;
    std::vector<float> dist_v;

    // set_pose, way_pose
    float radius = r_speed;
    float dist = 0;

    dist_v.push_back(way.pose.position.x - my.pose.position.x);
    dist_v.push_back(way.pose.position.y - my.pose.position.y);
    dist_v.push_back(way.pose.position.z - my.pose.position.z);

    dist = Norm(dist_v);

    if (dist >= radius)
    {
        set.pose.position.x = my.pose.position.x + ((dist_v[0] * radius) / dist);
        set.pose.position.y = my.pose.position.y + ((dist_v[1] * radius) / dist);
        set.pose.position.z = way.pose.position.z;
    }
    else
    {
        set.pose.position.x = way.pose.position.x;
        set.pose.position.y = way.pose.position.y;
        set.pose.position.z = way.pose.position.z;
    }

    set.pose.orientation.x = way.pose.orientation.x;
    set.pose.orientation.y = way.pose.orientation.y;
    set.pose.orientation.z = way.pose.orientation.z;
    set.pose.orientation.w = way.pose.orientation.w;

    set_pos_pub.publish(set);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, local_pose_cb);
    ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    set_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 100);
    // Arduino pub,sub
    ardu_pub = nh.advertise<std_msgs::Bool>
        ("winchStart", 10);

    ros::Subscriber ardu_sub = nh.subscribe<std_msgs::Bool>
		("winchEnd", 1, arduino_cb);

    ros::Publisher camera_pub = nh.advertise<rohang2::GpsPoint>("cam_state", 10); //pub message name
	ros::Subscriber cam_loc_sub = nh.subscribe<rohang2::CameraPoint>("cam_loc", 10, cameraCallback);

    //nh.param<float>("Height", height, 15);
    //nh.param<float>("Speed", r_speed, 3);
    p_nh.getParam("Height", height);
    ROS_INFO("Set Height: %f", height);
    p_nh.getParam("Speed", r_speed);
    ROS_INFO("Set Speed: %f", r_speed);
    p_nh.getParam("H_mission", h_mis);
    ROS_INFO("Mission Height: %f", h_mis);
    
    p_nh.getParam("Mode", Pos_Mode);
    if(Pos_Mode==1){
        ROS_INFO("Set LLH Coor.");
        p_nh.getParam("LLH_NoDrone", L_no);
        p_nh.getParam("LLH_Mission", L_mis);
        p_nh.getParam("LLH_Return", L_ret);
        p_nh.getParam("LLH_Ex", L_ex);
        ROS_INFO("No Drone[LLH]: %f, %f, %f", L_no[0], L_no[1], L_no[2]);
        ROS_INFO("Mission[LLH]: %f, %f, %f", L_mis[0], L_mis[1], L_mis[2]);
        ROS_INFO("Return[LLH]: %f, %f, %f", L_ret[0], L_ret[1], L_ret[2]);
    }
    else if (Pos_Mode==2){
        ROS_INFO("Set ECEF Coor.");
        p_nh.getParam("ECEF_NoDrone", E_no);
        p_nh.getParam("ECEF_Mission", E_mis);
        p_nh.getParam("ECEF_Return", E_ret);
        ROS_INFO("No Drone[ECEF]: %f, %f, %f", E_no[0], E_no[1], E_no[2]);
        ROS_INFO("Mission[ECEF]: %f, %f, %f", E_mis[0], E_mis[1], E_mis[2]);
        ROS_INFO("Return[ECEF]: %f, %f, %f", E_ret[0], E_ret[1], E_ret[2]);
    }
    else{
        ROS_INFO("Set EN Coor.");
        p_nh.getParam("EN_NoDrone", E_no);
        p_nh.getParam("EN_Mission", E_mis);
        p_nh.getParam("EN_Return", E_ret);
        ROS_INFO("No Drone[EN]: %f, %f", P_no[0], P_no[1]);
        ROS_INFO("Mission[EN]: %f, %f", P_mis[0], P_mis[1]);
        ROS_INFO("Return[EN]: %f, %f", P_ret[0], P_ret[1]);
    }
    
    //the setpoint publishing rate MUST be faster than 20Hz
    ros::Rate rate(5.0);

    //nh.param("pub_setpoints_traj/wn", wn, 1.0);
    //nh.param("pub_setpoints_traj/r", r, 1.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped home, way;

    home.pose = local_pose.pose;
    way.pose = local_pose.pose;

    // Euler -> Quat
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);

    camera.a = 320; camera.b = 240;

    //send a few setpoints before starting
    for (int i = 30; ros::ok() && i > 0; --i)
    {
        L_home[0] = global_pose.latitude;
        L_home[1] = global_pose.longitude;
        L_home[2] = global_pose.altitude;

        home.pose.position.x = local_pose.pose.position.x;
        home.pose.position.y = local_pose.pose.position.y;
        home.pose.position.z = height;

        home.pose.orientation.x = local_pose.pose.orientation.x;
        home.pose.orientation.y = local_pose.pose.orientation.y;
        home.pose.orientation.z = local_pose.pose.orientation.z;
        home.pose.orientation.w = local_pose.pose.orientation.w;

        set_pos_pub.publish(home);

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();

    // Frmae Tranceform
    if (Pos_Mode==1){
        // LLH -> ECEF
        E_home = llh2ecef(L_home);
        E_no = llh2ecef(L_no);
        E_mis = llh2ecef(L_mis);
        E_ret = llh2ecef(L_ret);
    }
    if (Pos_Mode!=0){
        // ECEF -> EN ( East, North )
        P_no = ecef2en(E_no, E_home);
        P_mis = ecef2en(E_mis, E_home);
        P_ret = ecef2en(E_ret, E_home);
    }
    if (f_NoEx){
        if (Pos_Mode==1)
            E_ex = llh2ecef(L_ex);
        if (Pos_Mode!=0)
            P_ex = ecef2en(E_ex, E_home);
    }

    int Number = 0;
    float dist=0, d_path;
    float r_path = 3.0f;
    std::vector<std::vector<float>> path, path2;
    int n = 0;

    way.pose.position.x = home.pose.position.x;
    way.pose.position.y = home.pose.position.y;
    way.pose.position.z = home.pose.position.z;
    way.pose.orientation.x = home.pose.orientation.x;
    way.pose.orientation.y = home.pose.orientation.y;
    way.pose.orientation.z = home.pose.orientation.z;
    way.pose.orientation.w = home.pose.orientation.w;

    while (ros::ok())
    {
        if (current_state.mode == "OFFBOARD")
        {
            switch (Number)
            {
            case 0: // Take off
                if(local_pose.pose.position.z > (height -2.0f)){
                // Next Level
                    Number = 1;
                }
                break;

            case 1: // go Home
                dist = dist_pose(P_home, local_pose);

                way.pose.position.x = P_home[0];
                way.pose.position.y = P_home[1];
                way.pose.position.z = height;

                // Next Level
                if (dist < r_area){
                    Number = 2;
                    path.clear();
                }
                break;

            case 2: // Go Mission Position (WPT#1)
                dist = dist_pose(P_mis, local_pose);
                
                if (path.size() == 0){
                    path = MakePath(P_home, P_mis, P_no, r_no);
                    if (f_NoEx){
                        path2 = MakePath(P_home, P_mis, P_ex, r_no);
                        if(path.size() < path2.size()){
                            path.clear();
                            path = path2;
                            path2.clear();
                        }
                    }
                    way.pose.position.x = path[0][0];
                    way.pose.position.y = path[0][1];
                    way.pose.position.z = height+0.01;
                    quat.setRPY(0, 0, DEF_D2R(0));
                    way.pose.orientation.x = quat[0];
                    way.pose.orientation.y = quat[1];
                    way.pose.orientation.z = quat[2];
                    way.pose.orientation.w = quat[3];
                }
                else{
                    d_path = dist_pose(path[n], local_pose);
                }


                // Arrval Path point
                if (d_path < r_path && (n<path.size()-1)){
                    n++;
                    way.pose.position.x = path[n][0];
                    way.pose.position.y = path[n][1];
                    way.pose.position.z = height;
                }

                // Next Level
                if (dist < r_area){
                    Number = 3;
                    path.clear();
                    n = 0;
                }

                break;

            case 3: // Mission Start
                way.pose.position.x = P_mis[0]+Correct[0];
                way.pose.position.y = P_mis[1]+Correct[1];
                way.pose.position.z = h_mis+0.01;
                quat.setRPY(0, 0, DEF_D2R(0 +Correct[2]));
                way.pose.orientation.x = quat[0];
                way.pose.orientation.y = quat[1];
                way.pose.orientation.z = quat[2];
                way.pose.orientation.w = quat[3];

                if (local_pose.pose.position.z < 15.0 && local_pose.pose.position.z >= 7.0) {

                    if(CamCount == 0){
                        gps.state=1;
                        camera_pub.publish(gps);
                        CamCount++;
                    }

                    if (camera.a < 320 - alpha) Correct[0] += 0.05;
                    else if (camera.a > 320 + alpha) Correct[0] -= 0.05;
                    camera.a = 320;
                    Correct[0] = 0;

                    if (camera.b < 240 - alpha) Correct[1] -= 0.05;
                    else if (camera.b > 240 + alpha) Correct[1] += 0.05; 
                    camera.b = 240;  
                    Correct[1] = 0;                
                }

                if (local_pose.pose.position.z < 7.0) {
                        gps.state=0;
				        camera_pub.publish(gps);
                        CamCount--;
                }

                if(local_pose.pose.position.z <= h_mis+0.2f){
                    if(arduPubCount == 0){
                        arduino_state.data = true;
                        ardu_pub.publish(arduino_state);
                        arduPubCount++;
                    }
                }

                // Next Level
                if (f_mis){
                    Number = 4;
                    arduPubCount = 0;
                    f_mis = false;
                    
                }
                break;

            case 4: // Take-off
                way.pose.position.x = P_mis[0];
                way.pose.position.y = P_mis[1];
                way.pose.position.z = height+0.01;

                if (local_pose.pose.position.z > (height - 2.0f)){
                    // Next Level
                    Number = 5;
                }

                break;

            case 5: // Go return Position (WPT#2)
                dist = dist_pose(P_ret, local_pose);
                
                if (path.size() == 0){
                    path = MakePath(P_mis, P_ret, P_ex, r_no);
                    if (f_NoEx){
                        path2 = MakePath(P_mis, P_ret, P_ex, r_no);
                        ROS_WARN("Path Size: %f %f", (float)path.size(),(float)path2.size());
                        if(path.size() < path2.size()){
                            path.clear();
                            path = path2;
                            path2.clear();
                        }
                    }
                    way.pose.position.x = path[0][0];
                    way.pose.position.y = path[0][1];
                    way.pose.position.z = height+0.02;
                    quat.setRPY(0, 0, DEF_D2R(-90));
                    way.pose.orientation.x = quat[0];
                    way.pose.orientation.y = quat[1];
                    way.pose.orientation.z = quat[2];
                    way.pose.orientation.w = quat[3];
                }
                else
                    d_path = dist_pose(path[n], local_pose);

                // Arrval Path point
                if (d_path < r_path && (n<path.size()-1)){
                    n++;
                    way.pose.position.x = path[n][0];
                    way.pose.position.y = path[n][1];
                    way.pose.position.z = height;
                }

                // Next Level
                if (dist < r_area){
                    Number = 6;
                    path.clear();
                    n = 0;
                }

                break;

            case 6: // Go Mission Position (WPT#3)
                dist = dist_pose(P_mis, local_pose);
                
                if (path.size() == 0){
                    path = MakePath(P_ret, P_mis, P_no, r_no);
                    if (f_NoEx){
                        path2 = MakePath(P_ret, P_mis, P_ex, r_no);
                        if(path.size() < path2.size()){
                            path.clear();
                            path = path2;
                            path2.clear();
                        }
                    }
                    way.pose.position.x = path[0][0];
                    way.pose.position.y = path[0][1];
                    way.pose.position.z = height+0.03;
                    quat.setRPY(0, 0, DEF_D2R(-90));
                    way.pose.orientation.x = quat[0];
                    way.pose.orientation.y = quat[1];
                    way.pose.orientation.z = quat[2];
                    way.pose.orientation.w = quat[3];
                }
                else
                    d_path = dist_pose(path[n], local_pose);

                // Arrval Path point
                if (d_path < r_path && (n<path.size()-1)){
                    n++;
                    way.pose.position.x = path[n][0];
                    way.pose.position.y = path[n][1];
                    way.pose.position.z = height;
                }

                // Next Level
                if (dist < r_area){
                    Number = 7;
                    path.clear();
                    n = 0;
                }

                break;

            case 7: // Mission Start
                way.pose.position.x = P_mis[0] + Correct[0];
                way.pose.position.y = P_mis[1] + Correct[1];
                way.pose.position.z = h_mis+0.03;
                quat.setRPY(0, 0, DEF_D2R(-90 + Correct[3]));
                way.pose.orientation.x = quat[0];
                way.pose.orientation.y = quat[1];
                way.pose.orientation.z = quat[2];
                way.pose.orientation.w = quat[3];

                if (local_pose.pose.position.z < 15.0 && local_pose.pose.position.z >= 7.0) {

                    if(CamCount == 0){
                        gps.state=1;
                        camera_pub.publish(gps);
                        CamCount++;
                    }

                    if (camera.a < 320 - alpha) Correct[0] += 0.05;
                    else if (camera.a > 320 + alpha) Correct[0] -= 0.05;
                    camera.a = 320;
                    Correct[0] = 0;

                    if (camera.b < 240 - alpha) Correct[1] -= 0.05;
                    else if (camera.b > 240 + alpha) Correct[1] += 0.05; 
                    camera.b = 240;  
                    Correct[1] = 0;                
                }

                if (local_pose.pose.position.z < 7.0) {
                        gps.state=0;
				        camera_pub.publish(gps);
                        CamCount--;
                }

                if(local_pose.pose.position.z <= h_mis+0.2f){
                    if(arduPubCount == 0){
                        arduino_state.data = true;
                        ardu_pub.publish(arduino_state);
                        arduPubCount++;
                    }
                }

                // Next Level
                if (f_mis){
                    Number = 8;
                    arduPubCount = 0;
                    f_mis = false;
                }
                break;

            case 8: // Take-off
                way.pose.position.x = P_mis[0];
                way.pose.position.y = P_mis[1];
                way.pose.position.z = height+0.03;

                // Next Level
                if (local_pose.pose.position.z > (height - 2.0f)){
                    Number = 9;
                }
                break;

            case 9: // Go Home
                dist = dist_pose(P_home, local_pose);
                
                if (path.size() == 0){
                    path = MakePath(P_mis, P_home, P_no, r_no);
                    if (f_NoEx){
                        path2 = MakePath(P_mis, P_home, P_ex, r_no);
                        if(path.size() < path2.size()){
                            path.clear();
                            path = path2;
                            path2.clear();
                        }
                    }
                    way.pose.position.x = path[0][0];
                    way.pose.position.y = path[0][1];
                    way.pose.position.z = height;
                    way.pose.orientation.x = home.pose.orientation.x;
                    way.pose.orientation.y = home.pose.orientation.y;
                    way.pose.orientation.z = home.pose.orientation.z;
                    way.pose.orientation.w = home.pose.orientation.w;
                }
                else
                    d_path = dist_pose(path[n], local_pose);

                // Arrval Path point
                if (d_path < r_path && (n<path.size()-1)){
                    n++;
                    way.pose.position.x = path[n][0];
                    way.pose.position.y = path[n][1];
                    way.pose.position.z = height;
                }

                // Next Level
                if (dist < r_area){
                    Number = 10;
                    path.clear();
                    n = 0;
                }
                break;

            case 10: // Landing
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Auto Land enabled");
                }
                break;

            default:
                break;
            }
            
        }
        else
        {
            // Refresh Pose
            way.pose.position.x = local_pose.pose.position.x;
            way.pose.position.y = local_pose.pose.position.y;
            way.pose.position.z = height;
            Number = 0;
        }
        std::cout << "dist: " << dist;
        std::cout << "way: " <<way.pose.position.x<< " "<<way.pose.position.y;
        std::cout << "pose: " <<local_pose.pose.position.x<< " "<<local_pose.pose.position.y;
        std::cout << "  WP: " << Number << std::endl;
        set_setpoint(way, local_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
