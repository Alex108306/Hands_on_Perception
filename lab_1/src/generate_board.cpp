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

    if (argc < 7){
    std::cerr << "Usage: " << argv[0] << " <NUM_ROWS::int> <NUM_COLLUMS::int> <DICT_NAME> <MARKER_SIZE:int> <SEPERATION:int> <OUTPUT_FILE.png>" << std::endl;
    return -1;
  }
    float num_rows = float(stoi(argv[1]));
    float num_collumns = float(stoi(argv[2]));
    string dict_aruco_marker = argv[3];
    float marker_size = float(stoi(argv[4]));
    float seperation = float(stoi(argv[5]));
    string name_png_file = argv[6];

    cv::aruco::PredefinedDictionaryType dictType;

    if (str_to_dict_aruco(dict_aruco_marker, dictType) != true){
        cerr << "Invalid dictionary name: " << dict_aruco_marker << endl;
    }

    cv::Mat markerImg;

    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(dictType);

    cv::aruco::Board board_marker;

    board_marker = cv::aruco::GridBoard(cv::Size(num_collumns, num_rows), marker_size, seperation, dict);

    float total_width = num_collumns * marker_size + (num_collumns - 1) * seperation + 50;

    float total_height = num_rows * marker_size + (num_rows - 1) * seperation + 50;

    board_marker.generateImage(cv::Size(total_width, total_height), markerImg, 20);

    cv::imwrite(name_png_file, markerImg);

    return 0;

}