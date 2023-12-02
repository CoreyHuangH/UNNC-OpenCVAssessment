**Introduction**

This report will introduce CV Task 2 which is the tracking of armour plate in a video stream: provide a video and ask to identify and track all appearances of armour plate light bands in the video, and output the identification result and current angle, allowing errors but explain the source of errors and provide theoretical solutions.



**Code and Explanation**

Let's explain the code line be line.

```c++
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
```

This block includes necessary headers for working with ROS (Robot Operating System) messages, OpenCV, and image processing.

```c++
ros::Publisher image_publisher;
cv::Mat transformMatrix; // Make sure to initialize this matrix properly
```

Declaration of a ROS publisher for image messages and an OpenCV matrix to store the transformation matrix.

```c++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "armor_strip_detection_node");
    ros::NodeHandle nh;

    image_publisher = nh.advertise<sensor_msgs::Image>("aligned_image", 1);
```

Initialization of ROS node and creation of a publisher for an image topic named "aligned_image".

```c++
    cv::VideoCapture cap("/home/ubuntu/catkin_ws/src/armor_tracking/src/video/stream.mp4");

    if (!cap.isOpened())
    {
        ROS_ERROR("Could not open the video file");
        return -1;
    }
```

Creation of an OpenCV video capture object to read frames from a video file. If the file cannot be opened, it prints an error message and exits.

```c++
    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Aligned Image", cv::WINDOW_AUTOSIZE);
```

Creation of OpenCV windows for displaying the original and aligned images.

```c++
    while (ros::ok())
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
            break;
```

A loop that continues as long as ROS is okay and frames can be read from the video capture. It reads a frame and breaks out of the loop if the frame is empty.

```c++
        float rotation_angle = 45.0;
        float scale_factor = 1.0;

        transformMatrix = cv::getRotationMatrix2D(cv::Point2f(frame.cols / 2, frame.rows / 2), rotation_angle, scale_factor);
```

Defines rotation angle and scale factor, then calculates a 2D rotation matrix using OpenCV's `getRotationMatrix2D` function.

```c++
        cv::Mat alignedImage;
        cv::warpAffine(frame, alignedImage, transformMatrix, frame.size());
```

Applies the affine transformation to align the armor strip in the frame.

```c++
        cv::Mat hsvImage;
        cv::cvtColor(alignedImage, hsvImage, cv::COLOR_BGR2HSV);
```

Converts the aligned image to the HSV color space for better color-based segmentation.

```c++
        cv::Scalar lowerBlue = cv::Scalar(100, 100, 50);
        cv::Scalar upperBlue = cv::Scalar(140, 255, 255);
```

Defines a range for blue color in the HSV color space.

```c++
        cv::Mat blueMask;
        cv::inRange(hsvImage, lowerBlue, upperBlue, blueMask);
```

Creates a binary mask (`blueMask`) by thresholding the image based on the defined blue color range.

```c++
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
```

Finds contours in the binary mask using OpenCV's `findContours` function.

```c++
        for (const auto &contour : contours)
        {
            cv::Rect boundingRect = cv::boundingRect(contour);
            cv::rectangle(alignedImage, boundingRect, cv::Scalar(0, 255, 0), 2);

            cv::Point2f center = cv::Point2f(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);
            double angle = atan2(center.y - frame.rows / 2.0, center.x - frame.cols / 2.0) * 180.0 / CV_PI;

            std::ostringstream angleText;
            angleText << "Angle: " << angle;
            cv::putText(alignedImage, angleText.str(), cv::Point(boundingRect.x, boundingRect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }
```

Processes each contour: draws a rectangle around the detected blue region, calculates an angle based on the rectangle center, and displays the angle on the image.

```c++
        cv::imshow("Original Image", frame);
        cv::imshow("Aligned Image", alignedImage);
```

Displays the original and aligned images.

```c++
        if (image_publisher.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage cv_image;
            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image.image = alignedImage;
            image_publisher.publish(cv_image.toImageMsg());
        }
```

Publishes the aligned image as a ROS message if there are subscribers.

```c++
        ros::spinOnce();
        cv::waitKey(30);
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
```

Spins the ROS node once, waits for a key press, and releases the video capture object and destroys OpenCV windows before exiting the program.



**Summary**

Here's a summary of its main functionality: 

1. **Initialization**: The code initializes a ROS node, sets up a publisher for an image topic ("aligned_image"), and opens a video file for processing.

2. **Image Processing Loop**: The main loop reads frames from the video stream. For each frame, it performs the following steps:

   a. Applies an affine transformation to align the armor strip in the frame.

   b. Converts the aligned image to the HSV color space for better color-based segmentation.

   c. Defines a range for blue color in the HSV color space and creates a binary mask by thresholding the image.

   d. Finds contours in the binary mask to identify blue regions.

   e. Draws rectangles around the detected blue regions and calculates the angle of each region based on its center.

   f. Displays the original and aligned images with rectangles and angle information.

   g. Publishes the aligned image as a ROS message if there are subscribers.

3. **Cleanup**: After processing all frames, the code releases the video capture object and closes the OpenCV windows.

In summary, the code utilizes ROS for communication and OpenCV for image processing to detect and analyze blue regions (armor strips) in a video stream. The armor strips are aligned using an affine transformation, and information about their angles is displayed on the images and published as ROS messages.





