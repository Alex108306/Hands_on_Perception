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

        // draw results
        if(!marker_ids.empty() && marker_ids.size() == 2) {
            cv::aruco::drawDetectedMarkers(img, marker_corners, marker_ids);
            for(unsigned int i = 0; i < marker_ids.size(); i++){
                cv::drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 1.5f, 2);
            }
            float rel_pos_x = tvecs[1][0] - tvecs[0][0];
            float rel_pos_y = tvecs[1][1] - tvecs[0][1];
            float rel_pos_z = tvecs[1][2] - tvecs[0][2];
            string pos_x = cv::format("Relative pos x: %.3f", rel_pos_x);
            string pos_y = cv::format("Relative pos y: %.3f", rel_pos_y);
            string pos_z = cv::format("Relative pos z: %.3f", rel_pos_z);
            cv::putText(img, pos_x, cv::Point2f(0, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            cv::putText(img, pos_y, cv::Point2f(0, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, pos_z, cv::Point2f(0, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }

        // Show image
        cv::imshow("Pose Estimation", img);
        key = (char)cv::waitKey(1);
    }


    return 0;
}