#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <random>

struct Node {
    cv::Point point;
    int parent;
    double cost;
};

class RRTStar {
public:
    RRTStar(int rows, int cols, cv::Point start, cv::Point goal);
    void run();
    void drawMap();
    void setCellSize(int size_x, int size_y);

private:
    int rows, cols;
    int cellSize_x, cellSize_y;
    cv::Point start, goal;
    cv::Mat map;
    std::vector<Node> nodes;
    std::vector<cv::Point> obstacleCells;
    std::mt19937 rng;

    void generateObstacles();
    bool isFreeCell(int row, int col);
    bool collisionFree(cv::Point a, cv::Point b);
    int getNearestNode(cv::Point pt);
    void optimizePath(int newNodeIdx);
    void drawPath(int idx);
};

#endif 
