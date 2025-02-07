#include "picam_ros2/const.hpp"
#include "picam_ros2/calibration.hpp"
#include <opencv2/imgproc.hpp>

cv::Mat yuv420ToRgbCopy(const std::vector<AVBufferRef *>& planes, const std::vector<unsigned int>& strides, uint width, uint height) {

    // Create Y plane Mat with stride
    cv::Mat y_full(height, strides[0], CV_8UC1, planes[0]->data);

    // Create U and V plane Mats with stride, but only every other row
    cv::Mat u_full(height / 2, strides[1], CV_8UC1, planes[1]->data);
    cv::Mat v_full(height / 2, strides[2], CV_8UC1, planes[2]->data);

    // Resize U and V to match the full image dimensions
    cv::Mat u_resized, v_resized;
    cv::resize(u_full, u_resized, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
    cv::resize(v_full, v_resized, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);

    // Combine YUV planes
    std::vector<cv::Mat> yuv_channels = {y_full, u_resized, v_resized};
    cv::Mat yuv;
    cv::merge(yuv_channels, yuv);

    // Convert YUV420 to RGB
    cv::Mat rgb;
    cv::cvtColor(yuv, rgb, cv::COLOR_YUV2BGR);

    return rgb;
}

cv::Mat yuv420ToMonoCopy(const std::vector<AVBufferRef *>& planes, const std::vector<unsigned int>& strides, uint width, uint height) {

    // Create Y plane Mat with stride
    cv::Mat y_full(height, strides[0], CV_8UC1, planes[0]->data);
    cv::Mat mono = y_full(cv::Rect(0, 0, width, height)).clone();

    return mono;
}

std::vector<cv::Point3f> createObjectPoints(cv::Size pattern_size, float square_size)
{
    std::vector<cv::Point3f> objectPoints;
    for(int i = 0; i < pattern_size.height; ++i)
    {
        for(int j = 0; j < pattern_size.width; ++j)
        {
            objectPoints.push_back(cv::Point3f(j*square_size, i*square_size, 0));
        }
    }
    return objectPoints;
}

void calibrateCamera(const std::vector<cv::Mat>& images, cv::Size pattern_size, float square_size, sensor_msgs::msg::CameraInfo& camera_info) {
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    
    // Find chessboard corners in all images
    int i = 0;
    for (const auto& image : images) {
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(image, pattern_size, corners);
        if (found) {
            std::cout << GREEN << "Chessboard found in frame #" << i << CLR << std::endl;
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            imagePoints.push_back(corners);
            objectPoints.push_back(createObjectPoints(pattern_size, square_size));
        } else {
            std::cout << RED << "Chessboard not found in frame #" << i << CLR << std::endl;
        }
        ++i;
    }
    
    // Calibrate camera
    std::cout << YELLOW << "Calibrating..." << CLR << std::endl;
    cv::Mat cameraMatrix, distCoeffs, R, T;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(objectPoints, imagePoints, images[0].size(), cameraMatrix, distCoeffs, rvecs, tvecs);
    
    // Create CameraInfo structure
    camera_info.distortion_model = "plumb_bob";
    cv::Mat_<double> K = cameraMatrix;
    std::copy_n(K.begin(), 9, camera_info.k.begin());
    camera_info.d = std::vector<double>(distCoeffs.begin<double>(), distCoeffs.end<double>());
}