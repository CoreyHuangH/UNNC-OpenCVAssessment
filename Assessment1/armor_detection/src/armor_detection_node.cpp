#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

class ArmorDetection
{
public:
    ArmorDetection(ros::NodeHandle nh, const std::string &image_folder)
        : nh_(nh), image_folder_(image_folder)
    {
        // 设置ROS发布器
        result_pub_ = nh_.advertise<std_msgs::String>("/armor_detection/result", 1);

        // 加载图片列表
        image_list_ = loadImages();

        // 设置定时器，每隔一段时间处理一张图片
        timer_ = nh_.createTimer(ros::Duration(1.0), &ArmorDetection::processNextImage, this);
    }

    // 处理下一张图片
    void processNextImage(const ros::TimerEvent &event)
    {
        if (current_image_index_ < image_list_.size())
        {
            std::string image_path = image_folder_ + "/" + image_list_[current_image_index_];
            cv::Mat original_image = cv::imread(image_path);

            // 设置仿射变换的旋转角度
            double rotation_angle = 30.0; // 统一的旋转角度，可以根据需求调整

            // 构造仿射变换矩阵
            cv::Point2f center(original_image.cols / 2.0, original_image.rows / 2.0);
            cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, rotation_angle, 1.0);

            // 进行仿射变换
            cv::Mat rotated_image;
            cv::warpAffine(original_image, rotated_image, rotation_matrix, original_image.size());

            // 装甲板灯带识别和仿射变换
            cv::Mat processed_image = processImage(rotated_image);

            // 显示处理后的图像
            cv::imshow("Processed Image", processed_image);
            cv::waitKey(0); // 等待按键

            // 保存处理后的图像
            std::string output_image_path = image_folder_ + "/output_" + image_list_[current_image_index_];
            cv::imwrite(output_image_path, processed_image);

            // 发布识别结果
            std_msgs::String result_msg;
            result_msg.data = "Armor detected in image " + std::to_string(current_image_index_ + 1);
            result_pub_.publish(result_msg);

            current_image_index_++;
        }
        else
        {
            // 所有图片已处理完毕，停止节点
            ROS_INFO("All images processed. Shutting down node.");
            ros::shutdown();
        }
    }

    // 加载图片列表
    std::vector<std::string> loadImages()
    {
        std::vector<std::string> image_list;
        try
        {
            for (int i = 1; i <= 5; ++i)
            {
                // 构造图片文件名
                std::string image_name = "armor" + std::to_string(i) + ".jpg";
                image_list.push_back(image_name);
            }
        }
        catch (...)
        {
            ROS_ERROR("Error loading images from %s.", image_folder_.c_str());
        }
        return image_list;
    }

    // 旋转图像
    cv::Mat rotateImage(const cv::Mat &image, double angle)
    {
        cv::Point2f center(image.cols / 2.0, image.rows / 2.0);
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
        cv::Mat rotated_image;
        cv::warpAffine(image, rotated_image, rotation_matrix, image.size());
        return rotated_image;
    }

    // 装甲板灯带识别和仿射变换
    cv::Mat processImage(const cv::Mat &original_image)
    {
        // 将图像从BGR转换为HSV色彩空间
        cv::Mat hsv_image;
        cv::cvtColor(original_image, hsv_image, cv::COLOR_BGR2HSV);

        // 调整只保留红色的颜色范围
        cv::Scalar lower_red = cv::Scalar(0, 100, 100);
        cv::Scalar upper_red = cv::Scalar(2, 255, 255);

        // 合并两个范围
        cv::Mat binary_image;
        cv::inRange(hsv_image, lower_red, upper_red, binary_image);

        // 对二值图像进行形态学操作，以去除噪音和连接装甲板区域
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(binary_image, binary_image, cv::MORPH_CLOSE, kernel);

        // 再进行开运算，去除边缘的噪点
        cv::morphologyEx(binary_image, binary_image, cv::MORPH_OPEN, kernel);

        cv::GaussianBlur(binary_image, binary_image, cv::Size(7, 7), 0); // 调整高斯滤波内核大小

        // 去除小的连通区域
        // cv::Mat labels, stats, centroids;
        // int num_labels = cv::connectedComponentsWithStats(binary_image, labels, stats, centroids);
        // for (int i = 1; i < num_labels; ++i)
        // {
        //     int area = stats.at<int>(i, cv::CC_STAT_AREA);
        //     if (area < 100) // 根据需求调整最小连通区域的面积阈值
        //     {
        //         cv::Mat mask = (labels == i);
        //         binary_image.setTo(0, mask);
        //     }
        // }


        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 画出轮廓和扩大的矩形
        cv::Mat result_image = original_image.clone();
        int max_area = 11300;               // 根据需求调整最大装甲板面积阈值
        int min_area = 2000;                // 根据需求调整最小装甲板面积阈值
        std::vector<cv::Rect> armor_rects; // 用于保存装甲板矩形框

        for (const auto &contour : contours)
        {
            // 计算外接矩形
            cv::Rect bounding_rect = cv::boundingRect(contour);

            // 判断矩形面积是否小于最大面积阈值
            int rect_area = bounding_rect.width * bounding_rect.height;
            if (rect_area > min_area && rect_area < max_area)
            {
                // 是装甲板，保存矩形框
                armor_rects.push_back(bounding_rect);
            }
        }

        // 绘制保留的装甲板矩形框
        for (const auto &rect : armor_rects)
        {
            cv::rectangle(result_image, rect, cv::Scalar(0, 255, 0), 2);
        }

        return result_image;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher result_pub_;
    ros::Timer timer_;
    std::string image_folder_;
    std::vector<std::string> image_list_;
    size_t current_image_index_ = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armor_detection_node");
    ros::NodeHandle nh;

    if (argc != 2)
    {
        ROS_ERROR("Usage: rosrun your_package_name armor_detection_node ~/catkin_ws/src/your_package_name/images");
        return 1;
    }

    std::string image_folder = argv[1];

    ArmorDetection armor_detection(nh, image_folder);

    ros::spin();

    return 0;
}
