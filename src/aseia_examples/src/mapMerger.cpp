#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid static_map;
nav_msgs::OccupancyGrid merged_map;
ros::Publisher map_pub;

ros::Time last_pub;
const short DOOR_COUNT = 3;
const short ROBOT_COUNT = 1;
short door_merge_count = 0;
short robot_merge_count = 0;

void set_static_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    static_map.header.frame_id = msg->header.frame_id;

    static_map.info.map_load_time     = msg->info.map_load_time;
    static_map.info.resolution        = msg->info.resolution;
    static_map.info.width             = msg->info.width;
    static_map.info.height            = msg->info.height;
    static_map.info.origin.position.x = msg->info.origin.position.x;
    static_map.info.origin.position.y = msg->info.origin.position.y;

    static_map.data = msg->data;

    if(merged_map.data.empty()){
        merged_map = static_map;
    }
}

void merge(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    int x_offset = msg->info.origin.position.x;
    int y_offset = msg->info.origin.position.y;

    int map_width    = static_map.info.width;
    int map_height   = static_map.info.height;
    int patch_width  = msg->info.width;
    int patch_height = msg->info.height;

    for(int i = 0; i < patch_height; i++){
        int start_pos = y_offset * map_width + x_offset + i * map_height;
        std::cout << start_pos << std::endl;

        for(int j = 0; j < patch_width; j++){
            merged_map.data.at(start_pos + j) = std::max(merged_map.data.at(start_pos + j),
                                                    msg->data.at(i * patch_height + j));
        }
    }
}

// merge maps, when a patch map with a certain identifier has not been merged yet
void merge_maps(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    if(msg->header.frame_id == "door_map" && door_merge_count < DOOR_COUNT){
        merge(msg);
        door_merge_count++;
        std::cout << "merged door" << std::endl;
    }

    if(msg->header.frame_id == "robot_map" && robot_merge_count < ROBOT_COUNT){
        merge(msg); 
        robot_merge_count++;

        std::cout << "merged robot" << std::endl;
    }
}

void publish_map(const ros::TimerEvent&){

    // TODO: is this really necessary here or should this be done when merging?
    merged_map.header.stamp = ros::Time::now();
    map_pub.publish(merged_map);

    merged_map = static_map;
    door_merge_count  = 0;
    robot_merge_count = 0;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "grid_merger");
    ros::NodeHandle n;

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/merged_map", 10);
    ros::Subscriber staticMapSub = n.subscribe("/static_map", 1, set_static_map);
    ros::Subscriber patchMapSub  = n.subscribe("/patch_map", 1, merge_maps);

    ros::Timer publishMapTimer   = n.createTimer(ros::Duration(1.0), publish_map);

    ros::spin();

    return 0;
}