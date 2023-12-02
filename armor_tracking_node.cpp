#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

ros::Publisher image_publisher;
cv::Mat transformMatrix; // Make sure to initialize this matrix properly

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armor_strip_detection_node");
    ros::NodeHandle nh;

    image_publisher = nh.advertise<sensor_msgs::Image>("aligned_image", 1);

    cv::VideoCapture cap("/home/ubuntu/catkin_ws/src/armor_tracking/src/video/stream.mp4");

    if (!cap.isOpened())
    {
        ROS_ERROR("Could not open the video file");
        return -1;
    }

    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Aligned Image", cv::WINDOW_AUTOSIZE);

    while (ros::ok())
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
            break;

        // Perform affine transformation to align the armor strip
        float rotation_angle = 45.0; // Change this angle according to your requirement
        float scale_factor = 1.0;    // Change this scale factor according to your requirement

        // Initialize the transformMatrix
        transformMatrix = cv::getRotationMatrix2D(cv::Point2f(frame.cols / 2, frame.rows / 2), rotation_angle, scale_factor);

        cv::Mat alignedImage;
        cv::warpAffine(frame, alignedImage, transformMatrix, frame.size());

        // Convert to HSV color space for better color-based segmentation
        cv::Mat hsvImage;
        cv::cvtColor(alignedImage, hsvImage, cv::COLOR_BGR2HSV);

        // Define a range for blue color in HSV
        cv::Scalar lowerBlue = cv::Scalar(100, 100, 50);  // Adjust these values based on your actual color
        cv::Scalar upperBlue = cv::Scalar(140, 255, 255); // Adjust these values based on your actual color

        // Threshold the image to get the binary mask of the blue regions
        cv::Mat blueMask;
        cv::inRange(hsvImage, lowerBlue, upperBlue, blueMask);

        // Find contours in the binary mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw rectangles around the detected blue regions and calculate angles
        for (const auto &contour : contours)
        {
            cv::Rect boundingRect = cv::boundingRect(contour);
            cv::rectangle(alignedImage, boundingRect, cv::Scalar(0, 255, 0), 2);

            // Calculate angle based on the center of the rectangle
            cv::Point2f center = cv::Point2f(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);
            double angle = atan2(center.y - frame.rows / 2.0, center.x - frame.cols / 2.0) * 180.0 / CV_PI;

            // Display the angle on the image
            std::ostringstream angleText;
            angleText << "Angle: " << angle;
            cv::putText(alignedImage, angleText.str(), cv::Point(boundingRect.x, boundingRect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }

        // Display the original and aligned images
        cv::imshow("Original Image", frame);
        cv::imshow("Aligned Image", alignedImage);

        // Publish the aligned image if there are subscribers
        if (image_publisher.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage cv_image;
            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image.image = alignedImage;
            image_publisher.publish(cv_image.toImageMsg());
        }

        ros::spinOnce();

        // Add a delay to make sure the windows stay open
        cv::waitKey(30); // Adjust the delay as needed
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
