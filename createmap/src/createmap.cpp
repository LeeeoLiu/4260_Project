#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/bind.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

// Map parameter
const double GRID_RESOLUTION = 0.02;
const int MAP_SIZE_X = 400;
const int MAP_SIZE_Y = 400;
const int SCAN_THRESHOLD = 5;
const int DECAY_FACTOR = 1;

// Record the number of times the grid is scanned by the laser
std::vector<std::vector<int>> scan_count(MAP_SIZE_X, std::vector<int>(MAP_SIZE_Y, 0));

// Record whether it is confirmed as an obstacle
std::vector<std::vector<bool>> confirmed_obstacles(MAP_SIZE_X, std::vector<bool>(MAP_SIZE_Y, false));

// Record the number of times the grid is visited, if not, the value is -1
std::vector<std::vector<bool>> visited(MAP_SIZE_X, std::vector<bool>(MAP_SIZE_Y, false));

ros::Publisher map_publisher;
std::shared_ptr<tf::TransformListener> tf_listener;
std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;

bool map_origin_initialized = false;
tf::Vector3 map_origin_offset;

// center the map frame and odom frame
void broadcastMapFrame(const ros::Time& stamp)
{
    tf::Transform transform;
    if (map_origin_initialized)
    {
        transform.setOrigin(-map_origin_offset);
    } 
    else
    {
        transform.setOrigin(tf::Vector3(0, 0, 0));
    }
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster->sendTransform(tf::StampedTransform(transform, stamp, "map", "odom"));
}

// Using bresenhamLine to get the whole grid points on the line
std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> line;
    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {
        line.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        { 
            err += dy; 
            x0 += sx; 
        }
        if (e2 <= dx)
        { 
            err += dx; 
            y0 += sy; 
        }
    }
    return line;
}

// World (x, y) -> Gridmap (x, y)
std::pair<int, int> worldToGrid(double x, double y)
{
    if (map_origin_initialized)
    {
        x += map_origin_offset.x();
        y += map_origin_offset.y();
    }
    int gx = static_cast<int>(x / GRID_RESOLUTION) + MAP_SIZE_X / 2;
    int gy = static_cast<int>(y / GRID_RESOLUTION) + MAP_SIZE_Y / 2;
    return {gx, gy};
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ros::Time scan_time = scan->header.stamp;
    try {
        // Align time
        tf::StampedTransform laser_transform;
        tf_listener->lookupTransform("map", scan->header.frame_id, scan_time, laser_transform);

        tf::StampedTransform base_transform;
        tf_listener->lookupTransform("map", "base_footprint", scan_time, base_transform);

        // Initialize the map
        if (!map_origin_initialized) {
            map_origin_offset = base_transform.getOrigin();
            map_origin_initialized = true;
            ROS_INFO("Map origin initialized at robot starting position: (%.2f, %.2f)",
                     map_origin_offset.x(), map_origin_offset.y());
        }

        // Set range threshold and traverse all laser measurements
        const double min_range_threshold = 0.1;  // Minimum range threshold (10cm)
        const double max_range_threshold = scan->range_max;  // Max range from scan
        double angle = scan->angle_min;

        for (unsigned int i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
            double range = scan->ranges[i];

            // Ignore invalid or out-of-range measurements
            if (range < min_range_threshold || range > max_range_threshold) {
                continue;
            }

            // Get the start and end points of the laser beam
            double start_x = base_transform.getOrigin().x();
            double start_y = base_transform.getOrigin().y();
            double end_x = start_x + range * cos(angle + tf::getYaw(base_transform.getRotation()));
            double end_y = start_y + range * sin(angle + tf::getYaw(base_transform.getRotation()));

            // Convert world coordinates to grid map indices
            std::pair<int, int> start_grid = worldToGrid(start_x, start_y);
            std::pair<int, int> end_grid = worldToGrid(end_x, end_y);

            // Get all the grid cells along the laser beam
            std::vector<std::pair<int, int>> line_cells = bresenhamLine(start_grid.first, start_grid.second,
                                                                        end_grid.first, end_grid.second);

            // Update grid statuses
            for (size_t j = 0; j < line_cells.size(); ++j) {
                int gx = line_cells[j].first;
                int gy = line_cells[j].second;

                // Ensure the grid indices are within map bounds
                if (gx < 0 || gx >= MAP_SIZE_X || gy < 0 || gy >= MAP_SIZE_Y) {
                    continue;
                }

                // Mark visited
                visited[gx][gy] = true;

                // For the last cell (end of laser beam), mark as obstacle
                if (j == line_cells.size() - 1) {
                    scan_count[gx][gy]++;
                    if (scan_count[gx][gy] >= SCAN_THRESHOLD) {
                        confirmed_obstacles[gx][gy] = true;
                    }
                } else {
                    // For other cells, mark as free space
                    scan_count[gx][gy] = std::max(scan_count[gx][gy] - DECAY_FACTOR, 0);
                    confirmed_obstacles[gx][gy] = false;
                }
            }
        }
    } catch (const tf::TransformException& ex) {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }
}

void publishMap(const ros::TimerEvent& event) {
    ros::Time current_time = ros::Time::now();
    broadcastMapFrame(current_time);

    // Initialize the OccupancyGrid message
    nav_msgs::OccupancyGrid map_data;
    map_data.header.stamp = current_time;
    map_data.header.frame_id = "map";
    map_data.info.resolution = GRID_RESOLUTION;
    map_data.info.width = MAP_SIZE_X;
    map_data.info.height = MAP_SIZE_Y;

    // Set the origin (bottom-left corner of the map in world coordinates)
    map_data.info.origin.position.x = -map_origin_offset.x() - (MAP_SIZE_X / 2) * GRID_RESOLUTION;
    map_data.info.origin.position.y = -map_origin_offset.y() - (MAP_SIZE_Y / 2) * GRID_RESOLUTION;
    map_data.info.origin.position.z = 0;
    map_data.info.origin.orientation.w = 1.0;

    // Fill the map data based on grid states
    map_data.data.resize(MAP_SIZE_X * MAP_SIZE_Y, -1);  // Default to unknown (-1)

    for (int x = 0; x < MAP_SIZE_X; ++x) {
        for (int y = 0; y < MAP_SIZE_Y; ++y) {
            int index = y * MAP_SIZE_X + x;

            if (!visited[x][y]) {
                // Cell has not been visited, keep it as unknown
                map_data.data[index] = -1;
            } else if (confirmed_obstacles[x][y]) {
                // Cell is confirmed as an obstacle
                map_data.data[index] = 100;
            } else {
                // Cell is free space
                map_data.data[index] = 0;
            }
        }
    }

    // Publish the map
    map_publisher.publish(map_data);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "create_gridmap");
    ros::NodeHandle nh;

    tf_listener = std::make_shared<tf::TransformListener>();
    tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 50, true);

    // Using message_filter is to align the timestamps of /scan and /tf
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 10);
    tf::MessageFilter<sensor_msgs::LaserScan> tf_filter(scan_sub, *tf_listener, "map", 10, nh);
    tf_filter.setTolerance(ros::Duration(0.1));
    tf_filter.registerCallback(boost::bind(&scanCallback, _1));

    // set a timer to publish the map
    ros::Timer map_timer = nh.createTimer(ros::Duration(0.05), publishMap);
    publishMap(ros::TimerEvent());
    ros::spin();
    return 0;
}

