#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <queue>
#include <cmath>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

struct MapData
{
    nav_msgs::OccupancyGrid map;
    std::vector<int> inflated_map;
    bool has_map = false;
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double robot_radius = 0.20;
};

ros::Subscriber map_sub_;
ros::Subscriber goal_reached_sub_;
ros::Publisher goal_pub_;
ros::Publisher marker_pub_;
tf2_ros::Buffer tf_buffer_;  
tf2_ros::TransformListener* tf_listener_; 
MapData map_data_;
double last_stop_yaw_;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg);
void triggerExploration();
void buildInflatedObstacles();
void markReachableCells(int sx, int sy, std::vector<bool> &reachable);
void detectFrontiers(const std::vector<bool> &reachable, std::vector<std::pair<int, int>> &frontiers);
void clusterFrontiers(const std::vector<std::pair<int, int>> &frontier_cells, std::vector<std::vector<std::pair<int, int>>> &clusters);
int selectLargestCluster(const std::vector<std::vector<std::pair<int, int>>> &clusters);
bool getRobotPoseInMapFrame(double &x, double &y);
bool worldToMap(double wx, double wy, int &mx, int &my);
void mapToWorld(double mx, double my, double &wx, double &wy);

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    map_data_.map = *map_msg;
    map_data_.width = map_msg->info.width;
    map_data_.height = map_msg->info.height;
    map_data_.resolution = map_msg->info.resolution;
    map_data_.origin_x = map_msg->info.origin.position.x;
    map_data_.origin_y = map_msg->info.origin.position.y;
    map_data_.has_map = true;

    buildInflatedObstacles();

    ROS_INFO("Map received: w=%d, h=%d, resolution=%.3f; obstacles inflated.",
             map_data_.width, map_data_.height, map_data_.resolution);
}

void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        ROS_INFO("Goal reached. Triggering new frontier exploration.");
        triggerExploration();
    }
}

void triggerExploration()
{
    if (!map_data_.has_map)
    {
        ROS_WARN("No map data yet, skip frontier exploration.");
        return;
    }

    double rx, ry;
    if (!getRobotPoseInMapFrame(rx, ry))
    {
        ROS_WARN("Failed to get robot pose in map. Skip frontier exploration.");
        return;
    }

    int robot_mx, robot_my;
    if (!worldToMap(rx, ry, robot_mx, robot_my))
    {
        ROS_WARN("Robot position out of map bounds. Skip frontier exploration.");
        return;
    }

    std::vector<bool> reachable(map_data_.width * map_data_.height, false);
    markReachableCells(robot_mx, robot_my, reachable);

    std::vector<std::pair<int, int>> frontiers;
    detectFrontiers(reachable, frontiers);
    if (frontiers.empty())
    {
        ROS_INFO("No reachable frontier found. Possibly fully explored or blocked.");
        return;
    }

    std::vector<std::vector<std::pair<int, int>>> clusters;
    clusterFrontiers(frontiers, clusters);
    if (clusters.empty())
    {
        ROS_WARN("No valid cluster after frontier grouping!");
        return;
    }

    int best_idx = selectLargestCluster(clusters);
    const auto &best_cluster = clusters[best_idx];

    double sumx = 0.0, sumy = 0.0;
    for (const auto &c : best_cluster)
    {
        sumx += c.first;
        sumy += c.second;
    }
    sumx /= best_cluster.size();
    sumy /= best_cluster.size();

    double goal_wx, goal_wy;
    mapToWorld(sumx, sumy, goal_wx, goal_wy);

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_wx;
    goal.pose.position.y = goal_wy;
    goal.pose.orientation.w = 1.0;

    goal_pub_.publish(goal);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "frontier_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_wx;
    marker.pose.position.y = goal_wy;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.0);

    marker_pub_.publish(marker);

    ROS_INFO("Publish Frontier Goal => cluster_size=%zu, world=(%.2f, %.2f)",
             best_cluster.size(), goal_wx, goal_wy);
}

// Build an inflated obstacle map 
/*
TODO:
Just like the astar part, find the inflated area
*/
void buildInflatedObstacles()
{
    // Get the map properties
    int w = map_data_.width; 
    int h = map_data_.height;      
    double res = map_data_.resolution;  
    double r = map_data_.robot_radius;  

    // Compute the number of grid cells corresponding to the robot's radius
    const int inflate_cells = static_cast<int>(std::ceil(r / res));

    // Initialize the inflated map with zeros (empty space)
    map_data_.inflated_map.assign(w * h, 0);

    // 1. Iterate through the occupancy grid
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int idx = y * w + x;
            
            // If this cell is an obstacle
            if (map_data_.map.data[idx] > 50) {
                // 2. Expand obstacles using a square region
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        // Check boundaries
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                            int nidx = ny * w + nx;
                            map_data_.inflated_map[nidx] = 100; // Mark as inflated obstacle
                        }
                    }
                }
            }
        }
    }
}

void markReachableCells(int sx, int sy, std::vector<bool> &reachable)
{
    int w = map_data_.width;
    int h = map_data_.height;
    const auto &inflated_map = map_data_.inflated_map;

    auto toIndex = [&](int x, int y) { return y * w + x; };
    if (sx < 0 || sx >= w || sy < 0 || sy >= h)
    {
        ROS_WARN("Robot start cell out-of-bounds in cspace BFS.");
        return;
    }
    int s_idx = toIndex(sx, sy);
    std::queue<int> Q;
    Q.push(s_idx);
    reachable[s_idx] = true;

    static const int NX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int NY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    while (!Q.empty())
    {
        int curr = Q.front();
        Q.pop();
        int cx = curr % w;
        int cy = curr / w;

        for (int i = 0; i < 8; i++)
        {
            int nx = cx + NX[i];
            int ny = cy + NY[i];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;

            int n_idx = toIndex(nx, ny);
            if (!reachable[n_idx] && inflated_map[n_idx] == 0)
            {
                reachable[n_idx] = true;
                Q.push(n_idx);
            }
        }
    }
}

// Detect Frontiers
/*
TODO:
Traverse each point in the map
Check if there are unknown areas in the neighboring regions
*/
void detectFrontiers(const std::vector<bool> &reachable, std::vector<std::pair<int, int>> &frontiers)
{
    frontiers.clear();
    int w = map_data_.width;
    int h = map_data_.height;
    const auto &grid = map_data_.map.data;
    const auto &inflated_map = map_data_.inflated_map;

    auto indexOf = [&](int x, int y) { return y * w + x; };
    
    // Define 8-connected neighborhood offsets
    static const int dx8[] = {-1, -1, -1,  0,  0,  1, 1, 1};
    static const int dy8[] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // Traverse each point in the map
    for (int y = 1; y < h-1; ++y) {
        for (int x = 1; x < w-1; ++x) {
            int idx = indexOf(x, y);
            
            // Check if it's a free cell and reachable
            if (grid[idx] == 0 && inflated_map[idx] == 0 && reachable[idx]) {
                // Check neighbors for unknown cells (-1)
                bool is_frontier = false;
                for (int i = 0; i < 8; ++i) {
                    int nx = x + dx8[i];
                    int ny = y + dy8[i];
                    int nidx = indexOf(nx, ny);
                    
                    if (nx >= 0 && nx < w && ny >= 0 && ny < h && grid[nidx] == -1) {
                        is_frontier = true;
                        break;
                    }
                }
                
                if (is_frontier) {
                    frontiers.emplace_back(x, y);
                }
            }
        }
    }
}

// cluster frontiers
/*
TODO:
Traverse each frontier
Create a queue , add the current frontier point to the queue, and mark it as clustered
Remove a point from the queue and add it to the current cluster
If a neighbor is an unclustered frontier point and is reachable, add it to the queue and continue expanding
*/
void clusterFrontiers(const std::vector<std::pair<int, int>> &frontier_cells, std::vector<std::vector<std::pair<int, int>>> &clusters)
{
    clusters.clear();
    if (frontier_cells.empty())
        return;

    int w = map_data_.width;
    int h = map_data_.height;
    std::vector<bool> visited(w * h, false);

    auto toIndex = [&](int x, int y) { return y * w + x; };
    for (const auto &f : frontier_cells) {
        visited[toIndex(f.first, f.second)] = true;
    }

    static const int NX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int NY[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    std::vector<bool> clustered(w * h, false);

    // Cluster frontiers using BFS
    for (const auto &seed : frontier_cells) {
        int seed_idx = toIndex(seed.first, seed.second);
        
        if (clustered[seed_idx]) continue; // Skip if already clustered
        
        std::vector<std::pair<int, int>> cluster;
        std::queue<std::pair<int, int>> q;
        q.push(seed);
        clustered[seed_idx] = true;
        
        while (!q.empty()) {
            auto current = q.front();
            q.pop();
            cluster.push_back(current);
            
            // Check all 8 neighbors
            for (int i = 0; i < 8; ++i) {
                int nx = current.first + NX[i];
                int ny = current.second + NY[i];
                
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                
                int nidx = toIndex(nx, ny);
                if (visited[nidx] && !clustered[nidx]) {
                    clustered[nidx] = true;
                    q.emplace(nx, ny);
                }
            }
        }
        
        if (!cluster.empty()) {
            clusters.push_back(cluster);
        }
    }
}

/*
TODO:
1. Initialize tracking variables
2. Iterate through all clusters
3. Compare cluster sizes
4. Return the index of the largest cluster
*/
int selectLargestCluster(const std::vector<std::vector<std::pair<int, int>>> &clusters)
{
    if (clusters.empty()) return -1;
    
    int largest_index = 0;
    size_t largest_size = clusters[0].size();
    
    for (size_t i = 1; i < clusters.size(); ++i) {
        if (clusters[i].size() > largest_size) {
            largest_size = clusters[i].size();
            largest_index = i;
        }
    }
    
    return largest_index;
}

bool getRobotPoseInMapFrame(double &x, double &y)
{
    try
    {
        geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform("map", "base_footprint", ros::Time(0));
        x = tf_stamped.transform.translation.x;
        y = tf_stamped.transform.translation.y;
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("TF lookup (map->base_footprint) failed: %s", ex.what());
        return false;
    }
}

bool worldToMap(double wx, double wy, int &mx, int &my)
{
    if (!map_data_.has_map)
        return false;
    double dx = wx - map_data_.origin_x;
    double dy = wy - map_data_.origin_y;
    mx = static_cast<int>(dx / map_data_.resolution);
    my = static_cast<int>(dy / map_data_.resolution);

    if (mx < 0 || mx >= map_data_.width || my < 0 || my >= map_data_.height)
        return false;
    return true;
}

void mapToWorld(double mx, double my, double &wx, double &wy)
{
    wx = map_data_.origin_x + (mx + 0.5) * map_data_.resolution;
    wy = map_data_.origin_y + (my + 0.5) * map_data_.resolution;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_explorer_cspace_edt_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // get the tf
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    map_sub_ = nh.subscribe("/map", 1, mapCallback);
    goal_reached_sub_ = nh.subscribe("goal_reached", 1, goalReachedCallback);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 1);

    private_nh.param<double>("robot_radius", map_data_.robot_radius, 0.20);

    ROS_INFO("FrontierGoalPublisher node with inflation (%.2fm) initialized.", map_data_.robot_radius);

    ros::spin();
    return 0;
}

