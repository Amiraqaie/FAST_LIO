#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Used only by the single-threaded mapping node's output callbacks.
class ScanColor
{
public:
    struct Sample {
        uint32_t rgb = 0x808080;
        uint8_t has_color = 0;
    };

    explicit ScanColor(rclcpp::Node &node)
    {
        auto intrinsics = node.declare_parameter<std::vector<double>>("color.intrinsics");
        auto rotation = node.declare_parameter<std::vector<double>>("color.rotation");
        auto translation = node.declare_parameter<std::vector<double>>("color.translation");
        max_dt = node.declare_parameter<double>("color.max_time_difference", 0.05);
        width = node.declare_parameter<int>("color.width", 0);
        height = node.declare_parameter<int>("color.height", 0);
        const auto finite = [](const std::vector<double> &v) {
            return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
        };
        if (intrinsics.size() != 4 || rotation.size() != 9 || translation.size() != 3 ||
            !finite(intrinsics) || !finite(rotation) || !finite(translation) ||
            intrinsics[0] <= 0 || intrinsics[1] <= 0 || width <= 0 || height <= 0 ||
            !std::isfinite(max_dt) || max_dt < 0)
            throw std::invalid_argument("Invalid FAST-LIO color camera calibration");
        std::copy(intrinsics.begin(), intrinsics.end(), k.begin());
        std::copy(rotation.begin(), rotation.end(), r.begin());
        std::copy(translation.begin(), translation.end(), t.begin());
        auto topic = node.declare_parameter<std::string>("color.image_topic", "/camera/image_raw");
        auto qos = rclcpp::SensorDataQoS();
        if (node.declare_parameter<bool>("color.reliable_image", false)) qos.reliable();
        sub = node.create_subscription<sensor_msgs::msg::Image>(topic, qos,
            [this, logger = node.get_logger(), clock = node.get_clock()](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                if (msg->width != static_cast<unsigned>(width) || msg->height != static_cast<unsigned>(height)) {
                    RCLCPP_WARN_THROTTLE(logger, *clock, 5000, "Image size differs from color calibration");
                    return;
                }
                try {
                    images.push_back(cv_bridge::toCvShare(msg, "rgb8"));
                    if (images.size() > 16) images.pop_front();
                } catch (const cv_bridge::Exception &e) {
                    RCLCPP_WARN_THROTTLE(logger, *clock, 5000, "Cannot read color image: %s", e.what());
                }
            });
    }

    bool select(double scan_end)
    {
        selected.reset();
        double best = max_dt;
        for (const auto &image : images) {
            double dt = std::abs(rclcpp::Time(image->header.stamp).seconds() - scan_end);
            if (dt <= best) {
                selected = image;
                best = dt;
            }
        }
        return static_cast<bool>(selected);
    }

    template<class Point>
    Sample sample(const Point &p) const
    {
        if (!selected) return {};
        const double z = r[6]*p.x + r[7]*p.y + r[8]*p.z + t[2];
        if (!(z > 0)) return {};
        const double u = k[0]*(r[0]*p.x + r[1]*p.y + r[2]*p.z + t[0])/z + k[2];
        const double v = k[1]*(r[3]*p.x + r[4]*p.y + r[5]*p.z + t[1])/z + k[3];
        if (!(u >= 0 && u < width && v >= 0 && v < height)) return {};
        const auto pixel = selected->image.at<cv::Vec3b>(static_cast<int>(v), static_cast<int>(u));
        return {(uint32_t(pixel[0]) << 16) | (uint32_t(pixel[1]) << 8) | pixel[2], 1};
    }

    // The publisher supplies an unorganized, tightly row-packed PCL message.
    template<class ColorAt>
    static void append(sensor_msgs::msg::PointCloud2 &msg, ColorAt color_at)
    {
        const auto old_step = msg.point_step;
        sensor_msgs::msg::PointField rgb;
        rgb.name = "rgb";
        rgb.offset = old_step;
        rgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
        rgb.count = 1;
        msg.fields.push_back(rgb);
        auto valid = rgb;
        valid.name = "has_color";
        valid.offset += sizeof(uint32_t);
        valid.datatype = sensor_msgs::msg::PointField::UINT8;
        msg.fields.push_back(valid);
        msg.point_step += 8;  // Keep subsequent XYZ fields aligned.
        msg.row_step = msg.width * msg.point_step;
        msg.data.resize(size_t(msg.height) * msg.row_step);
        for (size_t n = size_t(msg.width) * msg.height; n > 0; --n) {
            const size_t i = n - 1;
            auto *dst = msg.data.data() + i * msg.point_step;
            std::memmove(dst, msg.data.data() + i * old_step, old_step);
            const Sample sample = color_at(i);
            std::memcpy(dst + rgb.offset, &sample.rgb, sizeof(sample.rgb));
            std::memset(dst + valid.offset, 0, 4);
            dst[valid.offset] = sample.has_color;
        }
    }

private:
    int width, height;
    double max_dt;
    std::array<double, 4> k;
    std::array<double, 9> r;
    std::array<double, 3> t;
    std::deque<cv_bridge::CvImageConstPtr> images;
    cv_bridge::CvImageConstPtr selected;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
};
