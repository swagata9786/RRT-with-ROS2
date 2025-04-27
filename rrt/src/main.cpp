#include <rclcpp/rclcpp.hpp>
#include "rrt/rrt_star.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rrt_star_node");

    // Declare and get ROS2 parameters
    int rows = 30, cols = 30;
    int start_cell_x = 2, start_cell_y = 2;
    int goal_cell_x = 27, goal_cell_y = 27;

    node->declare_parameter("rows", rows);
    node->declare_parameter("cols", cols);
    node->declare_parameter("start_cell_x", start_cell_x);
    node->declare_parameter("start_cell_y", start_cell_y);
    node->declare_parameter("goal_cell_x", goal_cell_x);
    node->declare_parameter("goal_cell_y", goal_cell_y);

    node->get_parameter("rows", rows);
    node->get_parameter("cols", cols);
    node->get_parameter("start_cell_x", start_cell_x);
    node->get_parameter("start_cell_y", start_cell_y);
    node->get_parameter("goal_cell_x", goal_cell_x);
    node->get_parameter("goal_cell_y", goal_cell_y);

    const int mapWidth = 600;
    const int mapHeight = 600;
    int cellSize_x = mapHeight / rows;
    int cellSize_y = mapWidth / cols;

    // Convert grid cell coordinates to pixel coordinates
    cv::Point start(((start_cell_x * cellSize_y)-(cellSize_y /2)), ((start_cell_y * cellSize_x)-(cellSize_x /2)));
    cv::Point goal(((goal_cell_x * cellSize_y)-(cellSize_y /2)), ((goal_cell_y * cellSize_x)-(cellSize_y /2)));

    // Debug output
    RCLCPP_INFO(node->get_logger(), "Grid Size: %d rows x %d cols", rows, cols);
    RCLCPP_INFO(node->get_logger(), "Cell size: x = %d, y = %d", cellSize_x, cellSize_y);
    RCLCPP_INFO(node->get_logger(), "Start (cell): (%d, %d) -> (pixel): (%d, %d)", start_cell_x, start_cell_y, start.x, start.y);
    RCLCPP_INFO(node->get_logger(), "Goal  (cell): (%d, %d) -> (pixel): (%d, %d)", goal_cell_x, goal_cell_y, goal.x, goal.y);

    // Initialize RRT*
    RRTStar rrt(rows, cols, start, goal);
    rrt.setCellSize(cellSize_x, cellSize_y);
    rrt.run();

    rclcpp::shutdown();
    return 0;
}



