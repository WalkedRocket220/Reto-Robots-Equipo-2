// Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PointStamped.h"
// Including C++ nominal libraries
#include <iostream>
// Including OpenCV libraries
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/videoio.hpp"

// Declaring global variables
cv::Mat frame, hsv_frame, R_frame, B_frame, G_frame, Y_frame, frame_HSV, rojo_threshold, azul_threshold, verde_threshold, amarillo_threshold;
cv::Mat rojo_opening, verde_opening, amarillo_opening, azul_opening, gaussian;

ros::Publisher pub_red;
ros::Publisher pub_purple;
ros::Publisher pub_green;
ros::Publisher pub_yellow;

// Counters for specific shapes
int red_square_count = 0;
int purple_square_count = 0;
int yellow_square_count = 0;
int green_square_count = 0;

using namespace cv;
const int max_value_H = 360 / 2;
const int max_value = 255;

// Red Detection
int R_low_H = 0, R_low_S = 233, R_low_V = 241;
int R_high_H = 1, R_high_S = max_value, R_high_V = max_value;

// Purple Detection
int B_low_H = 143, B_low_S = 233, B_low_V = 241;
int B_high_H = 159, B_high_S = max_value, B_high_V = max_value;

// Green Detection
int G_low_H = 54, G_low_S = 233, G_low_V = 241;
int G_high_H = 64, G_high_S = max_value, G_high_V = max_value;

// Yellow Detection
int Y_low_H = 27, Y_low_S = 233, Y_low_V = 241;
int Y_high_H = 37, Y_high_S = max_value, Y_high_V = max_value;

double estimateDistance(const std::vector<cv::Point>& contour) {
    // Get the bounding box of the contour
    cv::Rect boundRect = cv::boundingRect(contour);

    // Use the larger dimension (height or width) to avoid underestimation
    double pixel_size = std::max(boundRect.width, boundRect.height);

    // Focal length in pixels (from camera_info topic)
    double focal_length_px = 462.137;  // Focal length in pixels

    // Determine the real-world size based on pixel size (empirical threshold)
    double object_real_size;
    if (pixel_size < 50) {  // Adjust this threshold empirically
        object_real_size = 0.04685;  // Small cube size in meters
    } else {
        object_real_size = 0.0937;  // Large cube size in meters
    }

    // Calculate the distance using the formula: Z = (f * H_real) / H_pixel
    double distance = (focal_length_px * object_real_size) / pixel_size;

    return distance;
}


///////////////////// ROS Subscribers //////////////////////////////
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // Show the original frame
    cv::waitKey(1);

    //preprocess
    GaussianBlur(frame, gaussian, Size(5, 5), 0);

    // Convert to HSV
    cv::cvtColor(gaussian, hsv_frame, cv::COLOR_BGR2HSV);
}

///////////////////// Shape Identification Function //////////////////////
std::string identifyShape(const std::vector<cv::Point>& contour) {
    std::string shape = "Unidentified";
    double perimeter = cv::arcLength(contour, true); // Compute perimeter
    std::vector<cv::Point> approx;
    
    // Approximate the contour with polygons
    cv::approxPolyDP(contour, approx, 0.04 * perimeter, true);

    int vertices = approx.size(); // Number of vertices

    // Classify based on vertices
    if (vertices == 3) {
        shape = "Triangle";
    } else if (vertices == 4) {
        shape = "Cuadrado";  // Square
    }
    return shape;
}

//////////////////// Centroid Calculation Function //////////////////////
cv::Point2f calculateCentroid(const std::vector<cv::Point>& contour) {
    cv::Moments M = cv::moments(contour);  // Calculate the moments of the contour

    // Avoid division by zero
    if (M.m00 != 0) {
        int cx = static_cast<int>(M.m10 / M.m00);  // x-coordinate of centroid
        int cy = static_cast<int>(M.m01 / M.m00);  // y-coordinate of centroid
        return cv::Point2f(cx, cy);
    } else {
        return cv::Point2f(-1, -1);  // Invalid centroid
    }
}

void publishMetricCoordinates(ros::Publisher& pub, const std::string& frame_id, const cv::Point2f& centroid, double z) {
    // Intrinsic values from the RealSense camera
    double fx = 462.137; //pixels
    double fy = 462.137;
    double cx = 320.0; //pixels
    double cy = 240.0;
    double pixel_size = 1e-5;  // Test with a larger size (10 micrometers)

    // Shift origin to the center of the image
    double u_shifted = centroid.x - cx;
    double v_shifted = cy - centroid.y;  // Invert the y-axis

    // Convert pixel coordinates to metric coordinates
    double x_metric = (u_shifted * z) / fx;
    double y_metric = (v_shifted * z) / fy;

    // Publish the coordinates as a PointStamped message
    geometry_msgs::PointStamped point_msg;
    point_msg.header.frame_id = frame_id;
    point_msg.header.stamp = ros::Time::now();

    point_msg.point.x = x_metric;
    point_msg.point.y = y_metric;
    point_msg.point.z = z;

    pub.publish(point_msg);
}

//////////////////// Main Program ////////////////////////////////
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "camera_sub");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    // Create windows for displaying images
    //cv::namedWindow("Rojo");
    //cv::namedWindow("Morado");
    //cv::namedWindow("Verde");
    //cv::namedWindow("Amarillo");
    
    ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, &imageCallback);
    // Initialize the publishers
    pub_red = nh.advertise<geometry_msgs::PointStamped>("/polygon_coordinates/red", 10);
    pub_purple = nh.advertise<geometry_msgs::PointStamped>("/polygon_coordinates/purple", 10);
    pub_green = nh.advertise<geometry_msgs::PointStamped>("/polygon_coordinates/green", 10);
    pub_yellow = nh.advertise<geometry_msgs::PointStamped>("/polygon_coordinates/yellow", 10);


    while (ros::ok()) {
        if (!frame.empty()) {
            // Reset counters for each loop
            red_square_count = 0;
            purple_square_count = 0;
            yellow_square_count = 0;
            green_square_count = 0;

            cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

            // Detect and process RED objects
            inRange(frame_HSV, Scalar(R_low_H, R_low_S, R_low_V), Scalar(R_high_H, R_high_S, R_high_V), rojo_threshold);
            // Perform morphological opening
	        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
	        cv::morphologyEx(rojo_threshold, rojo_opening, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<Point>> red_contours;
            findContours(rojo_opening, red_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (size_t i = 0; i < red_contours.size(); i++) {
                std::string shape = identifyShape(red_contours[i]);
                if (shape == "Cuadrado") {
                    red_square_count++;  // Increment red square counter
                }
                // Calculate the centroid
                cv::Point2f centroid_rojo = calculateCentroid(red_contours[i]);

                // Estimate the distance
                double distance = estimateDistance(red_contours[i]);
                // Print the distance to the console
                //std::cout << "Red Square Distance: " << distance << " m" << std::endl;

                //drawContours(frame, red_contours, (int)i, Scalar(0, 0, 255), 2);
                if (centroid_rojo.x != -1 && centroid_rojo.y != -1) {
                circle(frame, centroid_rojo, 5, Scalar(0, 0, 0), -1);  // Draw centroid as a small circle
                publishMetricCoordinates(pub_red,"camera_color_optical_frame", centroid_rojo, distance);
                }
            }
            //imshow("Rojo", rojo_opening);

            // Detect and process Purple objects
            inRange(frame_HSV, Scalar(B_low_H, B_low_S, B_low_V), Scalar(B_high_H, B_high_S, B_high_V), azul_threshold);
            // Perform morphological opening
	        cv::morphologyEx(azul_threshold, azul_opening, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<Point>> purple_contours;
            findContours(azul_opening, purple_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (size_t i = 0; i < purple_contours.size(); i++) {
                std::string shape = identifyShape(purple_contours[i]);
                if (shape == "Cuadrado") {
                    purple_square_count++;  // Increment blue square counter
                }
                // Calculate the centroid
                cv::Point2f centroid_morado = calculateCentroid(purple_contours[i]);

                // Estimate the distance
                double distance = estimateDistance(purple_contours[i]);
                // Print the distance to the console
                //std::cout << "Purple Square Distance: " << distance << " m" << std::endl;

                //drawContours(frame, purple_contours, (int)i, Scalar(255, 0, 255), 2);
                if (centroid_morado.x != -1 && centroid_morado.y != -1) {
                circle(frame, centroid_morado, 5, Scalar(0, 0, 0), -1);  // Draw centroid as a small circle
                publishMetricCoordinates(pub_purple, "camera_color_optical_frame", centroid_morado, distance);
                }
            }
            //imshow("Morado", azul_opening);

            // Detect and process GREEN objects
            inRange(frame_HSV, Scalar(G_low_H, G_low_S, G_low_V), Scalar(G_high_H, G_high_S, G_high_V), verde_threshold);
            // Perform morphological opening
	        cv::morphologyEx(verde_threshold, verde_opening, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<Point>> green_contours;
            findContours(verde_opening, green_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (size_t i = 0; i < green_contours.size(); i++) {
                std::string shape = identifyShape(green_contours[i]);
                if (shape == "Cuadrado") {
                    green_square_count++;  // Increment green hexagon counter
                }
                // Calculate the centroid
                cv::Point2f centroid_verde = calculateCentroid(green_contours[i]);

                // Estimate the distance
                double distance = estimateDistance(green_contours[i]);
                // Print the distance to the console
                //std::cout << "Green Square Distance: " << distance << " m" << std::endl;

                //drawContours(frame, green_contours, (int)i, Scalar(0, 255, 0), 2);
                if (centroid_verde.x != -1 && centroid_verde.y != -1) {
                circle(frame, centroid_verde, 5, Scalar(0, 0, 0), -1);  // Draw centroid as a small circle
                publishMetricCoordinates(pub_green, "camera_color_optical_frame", centroid_verde, distance);
                }
            }
            //imshow("Verde", verde_opening);

            // Detect and process YELLOW objects
            inRange(frame_HSV, Scalar(Y_low_H, Y_low_S, Y_low_V), Scalar(Y_high_H, Y_high_S, Y_high_V), amarillo_threshold);
            // Perform morphological opening
	        cv::morphologyEx(amarillo_threshold, amarillo_opening, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<Point>> yellow_contours;
            findContours(amarillo_opening, yellow_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (size_t i = 0; i < yellow_contours.size(); i++) {
                std::string shape = identifyShape(yellow_contours[i]);
                if (shape == "Cuadrado") {
                    yellow_square_count++;  // Increment yellow hexagon counter
                }
                // Calculate the centroid
                cv::Point2f centroid_amarillo = calculateCentroid(yellow_contours[i]);

                // Estimate the distance
                double distance = estimateDistance(yellow_contours[i]);
                // Print the distance to the console
                //std::cout << "Yellow Square Distance: " << distance << " m" << std::endl;

                //drawContours(frame, yellow_contours, (int)i, Scalar(0, 255, 255), 2);
                if (centroid_amarillo.x != -1 && centroid_amarillo.y != -1) {
                circle(frame, centroid_amarillo, 5, Scalar(0, 0, 0), -1);  // Draw centroid as a small circle
                publishMetricCoordinates(pub_yellow, "camera_color_optical_frame", centroid_amarillo, distance);
                }
            }
            //imshow("Amarillo", amarillo_opening);

            // Overlay the counters for each shape on the video frame
            std::string text = "Red Squares: " + std::to_string(red_square_count);
            if (red_square_count > 0){
            putText(frame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
            }
            text = "Purple Squares: " + std::to_string(purple_square_count);
            if (purple_square_count > 0){
            putText(frame, text, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
            }
            text = "Green Squares: " + std::to_string(green_square_count);
            if (green_square_count > 0){
            putText(frame, text, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
            }
            text = "Yellow Squares: " + std::to_string(yellow_square_count);
            if (yellow_square_count > 0){
            putText(frame, text, Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
            }

            // Display the counters for each shape
            if (red_square_count > 0){
                std::cout << "Red Squares: " << red_square_count << std::endl;
            }
            if (purple_square_count > 0){
            std::cout << "Purple Squares: " << purple_square_count << std::endl;
            }
            if (green_square_count > 0){
            std::cout << "Green Squares: " << green_square_count << std::endl;
            }
            if (yellow_square_count > 0){
            std::cout << "Yellow Squares: " << yellow_square_count << std::endl;
            }
            // Show the original frame with contours
            cv::imshow("Contours", frame);
        } else {
            std::cout << "empty " << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
