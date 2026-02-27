#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/barcode.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <unordered_map>

#include <iostream>

// Defining function that map from string type to Dictionary type for opencv
bool str_to_dict_aruco(std::string &s, cv::aruco::PredefinedDictionaryType &out){
    std::unordered_map<std::string, cv::aruco::PredefinedDictionaryType> kdict = {
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
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " <DICT_NAME>" << std::endl;
    return -1;
  }

  cv::VideoCapture webCam(0);       // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one

  if (webCam.isOpened() == false){   // Check if the VideoCapture object has been correctly associated to the webcam
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
  }

  cv::Mat img;    // input image

  char charCheckForESCKey{0};

  std::string dict_aruco_marker = argv[1];

  cv::aruco::PredefinedDictionaryType dictType;

  if (str_to_dict_aruco(dict_aruco_marker, dictType) != true){
      std::cerr << "Invalid dictionary name: " << dict_aruco_marker << std::endl;
      return -1;
  }

  while (charCheckForESCKey != 27 && webCam.isOpened()){    // loop until ESC key is pressed or webcam is lost
    bool frameSuccess = webCam.read(img);           // get next frame from input stream

    if (!frameSuccess || img.empty()){              // if the frame was not read or read wrongly
      std::cerr << "error: Frame could not be read." << std::endl;
      break;
    }

    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(dictType);
    cv::aruco::DetectorParameters detector_params = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dict, detector_params);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    detector.detectMarkers(img, markerCorners, markerIds, rejectedCandidates);

    cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);


    // Declaration of windows for output video results. Check OpenCV docs for explanation of parameters
    cv::namedWindow("imgArucoDetected", cv::WINDOW_AUTOSIZE);

    // Show output video results windows
    cv::imshow("imgArucoDetected", img);

    charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
  }
  return 0;
}