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

int main(int argc, char *argv[]){
    
    // Get Input parameters
    if (argc < 7){
        cerr << "Usage:" << argv[0] << "<DICT NAME> <DETECTOR PARAMS> <NUM ROWS:int> <NUM COLLUMS:int> <LENGTH ARUCO:int> <SEPERATION:int> <NAME CALIBRATION PARAMETERS>" << endl;
        return -1;
    }
    string dict_name = argv[1];
    string detector_params = argv[2];
    int num_rows = stoi(argv[3]);
    int num_collums = stoi(argv[4]);
    float length_aruco = stof(argv[5]);
    float seperation = stof(argv[6]);
    string name_calibration_file = argv[7];
     
    // Initialize key for capturing image and counter of mimage
    char key{0};
    int i = 0;

    // Initialize marker ID and corner of aurco marker
    vector<int> Marker_IDs;
    vector<vector<cv::Point2f>> marker_corners, rejected_candidates;

    cv::Mat Original_Img;
    cv::Mat Aruco_detect_Img;
    
    cv::aruco::PredefinedDictionaryType dict_type;
    
    if (str_to_dict_aruco(dict_name, dict_type) != true){
        cerr << "Invalid dictionaey name:" << dict_name << endl;
    }
    
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(dict_type);
    cv::aruco::DetectorParameters detect_params = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dict, detect_params);

    cv::VideoCapture webCam(0);

    if (webCam.isOpened() == false){
        cerr << "error: Webcam could not be connected" << endl;
        return -1;
    }

    while (key != 27 && webCam.isOpened()){
        
        bool success = webCam.read(Original_Img);
        if (!success || Original_Img.empty()){
            cerr << "error : Frames cannot be read." << endl;
            break;
        }

        Aruco_detect_Img = Original_Img.clone();

        detector.detectMarkers(Aruco_detect_Img, marker_corners, Marker_IDs, rejected_candidates);

        cv::aruco::drawDetectedMarkers(Aruco_detect_Img, marker_corners, Marker_IDs);

        // Declaration of windows for output video results. Check OpenCV docs for explanation of parameters
        cv::namedWindow("imgArucoDetected", cv::WINDOW_AUTOSIZE);

        // Show output video results windows
        cv::imshow("imgArucoDetected", Aruco_detect_Img);

        key = cv::waitKey(1);  // gets the key pressed
        
        if (key == 'c'){
            cout << "Frame captured" << endl;
            string file_path = "../data/image_" + to_string(++i) + ".png";
            cv::imwrite(file_path, Original_Img);
        }

        if (i == 25){
            cout << "25 frames captured, exiting..." << endl;
            break;
        }
        
    }

    if (i < 25){
        cerr << "Not enough frames captured, exiting..." << endl;
        return -1;
    }

    cv::aruco::GridBoard grid_board(cv::Size(num_collums, num_rows), length_aruco, seperation, dict); // if switch to num_rows, num collums, the reprojection error will be lower
    cv::Mat camera_matrix, distCoeffs;
    vector<cv::Mat> rvecs, tvecs;
    vector<cv::Mat> processed_objectPoints, processed_imagePoints;
    cv::Size img_size;
    
    for (int j = 0; j < i; j++){

        string path_file = "../data/image_" + to_string(j+1) + ".png";
        cv::Mat img = cv::imread(path_file);
        img_size = img.size();

        detector.detectMarkers(img, marker_corners, Marker_IDs, rejected_candidates);

        cv::Mat current_objectPoints, current_imagePoints;
        grid_board.matchImagePoints(marker_corners, Marker_IDs, current_objectPoints, current_imagePoints);

        if(current_objectPoints.total() > 0 && current_imagePoints.total() > 0) {
            processed_objectPoints.push_back(current_objectPoints);
            processed_imagePoints.push_back(current_imagePoints);
        }
    }

    double repError = cv::calibrateCamera(processed_objectPoints, processed_imagePoints, img_size, camera_matrix, distCoeffs, rvecs, tvecs);
    
    // Store calibration parameters in yaml file
    string out_yaml = "../config/" + name_calibration_file + ".yaml";
    cv::FileStorage fs(out_yaml, cv::FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();

    cout << "Calibration complete" << endl;
    cout << "Reprojection error: " << repError << endl;
    cout << "Save yaml file to " << out_yaml << endl;

    return 0;

}