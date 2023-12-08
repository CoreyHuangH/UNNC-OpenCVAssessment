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

            // 装甲板灯带识别和仿射变换
            cv::Mat processed_image = processImage(original_image);

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
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::morphologyEx(binary_image, binary_image, cv::MORPH_CLOSE, kernel);

        // 再进行开运算，去除边缘的噪点
        cv::morphologyEx(binary_image, binary_image, cv::MORPH_OPEN, kernel);

        // 高斯滤波
        cv::GaussianBlur(binary_image, binary_image, cv::Size(17, 17), 0);

        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 画出轮廓和扩大的矩形
        cv::Mat result_image = original_image.clone();
        for (const auto &contour : contours)
        {
            // 计算外接矩形
            cv::Rect bounding_rect = cv::boundingRect(contour);

            // 扩大矩形
            int expansion_size = 10; // 根据需求调整扩大的大小
            bounding_rect.x -= expansion_size;
            bounding_rect.y -= expansion_size;
            bounding_rect.width += 2 * expansion_size;
            bounding_rect.height += 2 * expansion_size;

            // 画出扩大后的矩形
            cv::rectangle(result_image, bounding_rect, cv::Scalar(0, 255, 0), 2);
        }

        // 你可以在这里根据实际需求判断是否是装甲板，比如根据矩形的面积、长宽比等

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
