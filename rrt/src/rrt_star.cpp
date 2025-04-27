#include "rrt/rrt_star.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits>
#include <cmath>
#include <random>
#include <vector>

// Constructor: Initialize grid size, start/goal points, and default values
RRTStar::RRTStar(int rows, int cols, cv::Point start, cv::Point goal)
    : rows(rows), cols(cols), start(start), goal(goal), cellSize_x(20), cellSize_y(20), rng(1234) {
    map = cv::Mat::zeros(600, 600, CV_8UC3);
    generateObstacles();
    nodes.push_back({ start, -1, 0.0 });  // Start node with no parent and zero cost
}

// Set cell size for the grid
void RRTStar::setCellSize(int size_x, int size_y) {
    cellSize_x = size_x;
    cellSize_y = size_y;
}

// Generate obstacles randomly within the grid
void RRTStar::generateObstacles() {
    // Define full map size
    int mapWidth = 600;
    int mapHeight = 600;
    cellSize_x = mapHeight / rows;
    cellSize_y = mapWidth / cols;

    // Reset map to white
    map = cv::Mat(mapHeight, mapWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    obstacleCells.clear();

    // Random obstacle placement
    std::uniform_int_distribution<int> randRow(0, rows - 1);
    std::uniform_int_distribution<int> randCol(0, cols - 1);

    for (int i = 0; i < rows * cols / 5; ++i) {
        int r = randRow(rng);
        int c = randCol(rng);
        cv::Point cell(c, r);
        if (cell == start || cell == goal || !isFreeCell(r, c)) continue;

        obstacleCells.push_back(cell);

        // Draw obstacle aligned to grid
        cv::rectangle(
            map,
            cv::Rect(c * cellSize_y, r * cellSize_x, cellSize_y, cellSize_x),
            cv::Scalar(0, 0, 0),
            -1
        );
    }

    // Draw clean grid lines
    for (int r = 0; r <= rows; ++r)
        cv::line(map, cv::Point(0, r * cellSize_x), cv::Point(map.cols, r * cellSize_x), cv::Scalar(255, 150, 255));

    for (int c = 0; c <= cols; ++c)
        cv::line(map, cv::Point(c * cellSize_y, 0), cv::Point(c * cellSize_y, map.rows), cv::Scalar(255, 150, 255));
}

// Check if a cell is free (not an obstacle)
bool RRTStar::isFreeCell(int row, int col) {
    return std::find(obstacleCells.begin(), obstacleCells.end(), cv::Point(col, row)) == obstacleCells.end();
}

// Check if the line between two points is free of obstacles
bool RRTStar::collisionFree(cv::Point a, cv::Point b) {
    cv::LineIterator it(map, a, b);
    for (int i = 0; i < it.count; ++i, ++it) {
        cv::Vec3b color = map.at<cv::Vec3b>(it.pos());
        if (color == cv::Vec3b(0, 0, 0))  // Black pixel = obstacle
            return false;
    }
    return true;
}

// Get the nearest node index in the tree
int RRTStar::getNearestNode(cv::Point pt) {
    double minDist = std::numeric_limits<double>::max();
    int index = 0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        double dist = cv::norm(nodes[i].point - pt);
        if (dist < minDist) {
            minDist = dist;
            index = i;
        }
    }
    return index;
}

// Optimize the path by checking nearby nodes (Path Smoothing)
void RRTStar::optimizePath(int newNodeIdx) {
    double radius = 150.0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (cv::norm(nodes[i].point - nodes[newNodeIdx].point) < radius) {
            if (collisionFree(nodes[i].point, nodes[newNodeIdx].point)) {
                double newCost = nodes[i].cost + cv::norm(nodes[i].point - nodes[newNodeIdx].point);
                if (newCost < nodes[newNodeIdx].cost) {
                    nodes[newNodeIdx].parent = i;
                    nodes[newNodeIdx].cost = newCost;
                }
            }
        }
    }
}

// Draw the path from the goal to the start
void RRTStar::drawPath(int idx) {
    while (nodes[idx].parent != -1) {
        cv::line(map, nodes[idx].point, nodes[nodes[idx].parent].point, cv::Scalar(150, 100, 20), 2);
        idx = nodes[idx].parent;
    }
}

// Display the map with obstacles, start and goal points, and grid lines
void RRTStar::drawMap() {
    cv::Mat display = map.clone();

    // Draw start and goal
    cv::circle(display, start, 6, cv::Scalar(0, 255, 0), -1);  // Start: green
    cv::circle(display, goal, 6, cv::Scalar(0, 0, 255), -1);   // Goal: red

    // Draw clean grid lines on display only
    for (int r = 1; r < rows; ++r) {
        int y = r * cellSize_x;
        if (y < display.rows)
            cv::line(display, cv::Point(0, y), cv::Point(display.cols, y), cv::Scalar(255, 150, 255));
    }

    for (int c = 1; c < cols; ++c) {
        int x = c * cellSize_y;
        if (x < display.cols)
            cv::line(display, cv::Point(x, 0), cv::Point(x, display.rows), cv::Scalar(255, 150, 255));
    }

    cv::imshow("RRT* Grid", display);
    cv::resizeWindow("RRT* Grid", 600, 600);
    cv::waitKey(1);
}

// Main function to run the RRT* algorithm
void RRTStar::run() {
    std::uniform_int_distribution<int> randX(0, cols * cellSize_y - 1);
    std::uniform_int_distribution<int> randY(0, rows * cellSize_x - 1);

    for (int iter = 0; iter < 10000; ++iter) {
        cv::Point randPt = (iter % 10 == 0) ? goal : cv::Point(randX(rng), randY(rng));
        int nearest = getNearestNode(randPt);
        cv::Point dir = randPt - nodes[nearest].point;
        double len = cv::norm(dir);
        if (len > 30) dir = dir * (30.0 / len);
        cv::Point newPt = nodes[nearest].point + dir;

        if (!collisionFree(nodes[nearest].point, newPt)) continue;

        Node newNode = { newPt, nearest, nodes[nearest].cost + cv::norm(newPt - nodes[nearest].point) };
        nodes.push_back(newNode);
        int newIndex = nodes.size() - 1;

        cv::line(map, newPt, nodes[nearest].point, cv::Scalar(100, 0, 255), 1);
        cv::circle(map, newPt, 2, cv::Scalar(200, 200, 0), -1);

        optimizePath(newIndex);

        if (cv::norm(newPt - goal) < 15) {
            drawPath(newIndex);
            break;
        }

        if (iter % 100 == 0) drawMap();
    }

    drawMap();
    std::cout << "Press any key to close...\n";
    cv::waitKey(0);
}


