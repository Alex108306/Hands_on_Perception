#include <iostream>
#include <unordered_map>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/barcode.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

using namespace std;

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

int main(int argc, char* argv[]) {

    if (argc < 5){
    std::cerr << "Usage: " << argv[0] << " <DICT_NAME> <ID_MARKER:int> <MARKER_SIZE:int> <OUTPUT_FILE.png>" << std::endl;
    return -1;
  }

    string dict_aruco_marker = argv[1];
    int id_marker = stoi(argv[2]);
    int marker_size = stoi(argv[3]);
    string name_png_file = argv[4];

    cv::aruco::PredefinedDictionaryType dictType;

    if (str_to_dict_aruco(dict_aruco_marker, dictType) != true){
        cerr << "Invalid dictionary name: " << dict_aruco_marker << endl;
    }

    cv::Mat markerImg;

    cv::Mat markerPadded;

    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(dictType);

    cv::aruco::generateImageMarker(dict, id_marker, marker_size, markerImg, 1);

    cv::copyMakeBorder(markerImg, markerPadded, 20, 20, 20, 20, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    cv::imwrite(name_png_file, markerPadded);

    return 0;

}

