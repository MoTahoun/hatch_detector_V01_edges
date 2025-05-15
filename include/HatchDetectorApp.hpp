#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <sstream>
#include <arpa/inet.h>
#include <unistd.h>
#include <deque>

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>

class HatchDetectorApp
{
private:
    // Core logic
    void detectHatches(cv::Mat &display_image, cv::Mat &original_image, const sl::Mat &point_cloud);

    // Image processing
    void preprocessImage(cv::Mat &image, cv::Mat &output);
    std::vector<std::vector<cv::Point>> findContoursFromImage(const cv::Mat &edgeImage);
    bool isValidContour(const std::vector<cv::Point> &approx, const std::vector<cv::Point> &contour);

    // 3D processing
    std::vector<cv::Point3f> extract3DPoints(const sl::Mat &point_cloud, const cv::Rect &bbox);
    cv::Point3f compute3DCenter(const std::vector<cv::Point3f> &points);
    bool computePCAOrientation(const std::vector<cv::Point3f> &points, cv::Point3d &centroid, cv::Vec3d &eulerAngles, cv::Mat *eigen_out = nullptr);
    cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat &R);

    // Temporal smoothing
    void applyTemporalSmoothing(cv::Point3d &position, cv::Vec3d &orientation);
    std::deque<cv::Point3d> position_history;
    std::deque<cv::Vec3d> orientation_history;

    // ZED SDK components
    sl::Camera zed;
    sl::InitParameters init_params;
    sl::RuntimeParameters runtime_params;

public:
    HatchDetectorApp();
    ~HatchDetectorApp();

    bool initialize();
    void run();
};

// Global TCP pose sharing
extern std::mutex pose_mutex;
extern cv::Point3d latest_position;
extern cv::Vec3d latest_orientation;

// TCP background thread
void startTCPClient();

// TCP message helper methods
std::string formatFloat(float value, int width, int precision);
int computeChecksum(const std::string &body);