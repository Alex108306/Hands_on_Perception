#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/barcode.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " <video source no.>" << std::endl;
    return -1;
  }

  cv::VideoCapture webCam(std::atoi(argv[1]));       // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one

  if (webCam.isOpened() == false){   // Check if the VideoCapture object has been correctly associated to the webcam
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
  }

  cv::Mat img;    // input image

  char charCheckForESCKey{0};

  while (charCheckForESCKey != 27 && webCam.isOpened()){    // loop until ESC key is pressed or webcam is lost
    bool frameSuccess = webCam.read(img);           // get next frame from input stream

    if (!frameSuccess || img.empty()){              // if the frame was not read or read wrongly
      std::cerr << "error: Frame could not be read." << std::endl;
      break;
    }

    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
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