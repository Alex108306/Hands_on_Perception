#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/barcode.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <unordered_map>

#include <iostream>

using namespace std;

void draw_cube(cv::Mat img, vector<cv::Point2f> marker, cv::Mat camera_matrix, cv::Mat dist_coeffs, cv::Vec3d rvec, cv::Vec3d tvec, float marker_length){
    // Define the 3D points of the cube
    vector<cv::Point3f> cube_points = {
        cv::Point3f(-marker_length/2.f, marker_length/2.f, 0),
        cv::Point3f(marker_length/2.f, marker_length/2.f, 0),
        cv::Point3f(marker_length/2.f, -marker_length/2.f, 0),
        cv::Point3f(-marker_length/2.f, -marker_length/2.f, 0),
        cv::Point3f(-marker_length/2.f, marker_length/2.f, marker_length),
        cv::Point3f(marker_length/2.f, marker_length/2.f, marker_length),
        cv::Point3f(marker_length/2.f, -marker_length/2.f, marker_length),
        cv::Point3f(-marker_length/2.f, -marker_length/2.f, marker_length)
    };

    // Project the 3D points to 2D image points
    vector<cv::Point2f> image_points;
    cv::projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    // Draw the edges of the cube
    for (int i = 0; i < 4; i++) {
        cv::line(img, image_points[i], image_points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2); // Connect point 0-1, 1-2, 2-3, 3-0
        cv::line(img, image_points[i + 4], image_points[((i + 1) % 4) + 4], cv::Scalar(255, 0, 0), 2); // Connect 4-5, 5-6, 6-7, 7-4
        cv::line(img, image_points[i], image_points[i + 4], cv::Scalar(255, 0, 0), 2); // Connect 0-4, 1-5, 2-6, 3-7
    }
    

}

// Defining function that map from string type to Dictionary type for opencv
bool str_to_dict_aruco(string &s, cv::aruco::PredefinedDictionaryType &out){
    unordered_map<string, cv::aruco::PredefinedDictionaryType> kdict = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
    };

    out = kdict.at(s);
    return true;
}

int main(int argc, char* argv[]){

    if(argc < 4){
        cerr << "Usage:" << argv[0] << "<DICT NAME> <ID MARKER:int> <MARKER LENGTH:floar>" << endl;
    }

    // Store input from command line
    string dict_name = argv[1];
    int id_marker = stoi(argv[2]);
    float marker_length = stof(argv[3]);

    // Load camera calibration parameters
    cv::Mat camera_matrix, dist_coeffs;
    string camera_param_file = "../config/params.yaml";
    cv::FileStorage fs;
    fs.open(camera_param_file, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();

    // Set coordinate system
    cv::Mat obj_points(4, 1, CV_32FC3);
    obj_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length/2.f, marker_length/2.f, 0);
    obj_points.ptr<cv::Vec3f>(1)[0] = cv::Vec3f(marker_length/2.f, marker_length/2.f, 0);
    obj_points.ptr<cv::Vec3f>(2)[0] = cv::Vec3f(marker_length/2.f, -marker_length/2.f, 0);
    obj_points.ptr<cv::Vec3f>(3)[0] = cv::Vec3f(-marker_length/2.f, -marker_length/2.f, 0);

    // Initialize Aruco marker detector
    cv::aruco::PredefinedDictionaryType dict_type;
    if(!str_to_dict_aruco(dict_name, dict_type)){
        cerr << "Invalid dictionary name: " << dict_name << endl;
        return -1;
    }
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(dict_type);
    cv::aruco::DetectorParameters detect_params = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dict, detect_params);

    // Initialize variable to store image from webcam
    cv::Mat img;

    // Initialize webcam openning and key to close webcam
    cv::VideoCapture web_cam(0);
    char key{0};

    if (web_cam.isOpened() == false){
        cerr << "error: Webcam could not be connected" << endl;
        return -1;
    }

    while (key != 27 && web_cam.isOpened()){
        // Storing image from webcam
        bool success = web_cam.read(img);
        if(!success || img.empty()){
            cerr << "error: Frames cannot be read. " << endl;
        }

        // Detect markers and estimate pose
        vector<int> marker_ids;
        vector<vector<cv::Point2f>> marker_corners, rejected_candidates;
        detector.detectMarkers(img, marker_corners, marker_ids, rejected_candidates);

        size_t nMarkers = marker_corners.size();
        vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if(!marker_ids.empty()){
            // calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++){
                cv::solvePnP(obj_points, marker_corners.at(i), camera_matrix, dist_coeffs, rvecs.at(i), tvecs.at(i));
            }
        }

        // Draw cube
        if(!marker_ids.empty()) {
            for(unsigned int i = 0; i < marker_ids.size(); i++){
                draw_cube(img, marker_corners.at(i), camera_matrix, dist_coeffs, rvecs.at(i), tvecs.at(i), marker_length);
            }
        }

        // Show image
        cv::imshow("Pose Estimation", img);
        key = (char)cv::waitKey(1);
    }


    return 0;
}