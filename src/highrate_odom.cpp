#include <deque>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fast_lio/msg/lio_state.hpp>

using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using M23 = Eigen::Matrix<double, 23, 23>;
using M23x12 = Eigen::Matrix<double, 23, 12>;

namespace {
M3 skew(const V3 &v)
{
    M3 m;
    m <<     0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return m;
}
M3 expSO3(const V3 &w)
{
    double a = w.norm();
    if (a < 1e-12) return M3::Identity() + skew(w);
    return Eigen::AngleAxisd(a, w / a).toRotationMatrix();
}
}  // namespace

struct Imu { double t; V3 gyro; V3 acc; };

struct State {
    V3 pos = V3::Zero(), vel = V3::Zero(), bg = V3::Zero(), ba = V3::Zero(), grav = V3(0, 0, -9.81);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

class HighRateOdom : public rclcpp::Node
{
public:
    HighRateOdom() : Node("lio_highrate_odom")
    {
        imu_topic_   = declare_parameter<std::string>("imu_topic", "/livox/imu");
        state_topic_ = declare_parameter<std::string>("state_topic", "/lio_state");
        odom_topic_  = declare_parameter<std::string>("odom_topic", "/Odometry_imu");
        frame_id_    = declare_parameter<std::string>("frame_id", "odom");
        child_id_    = declare_parameter<std::string>("child_frame_id", "livox_frame");
        // Must match FAST-LIO: common.time_offset_lidar_to_imu (FAST-LIO does stamp = imu - offset)
        time_offset_ = declare_parameter<double>("time_offset_lidar_to_imu", 0.0);
        // FAST-LIO scales acc by G / |mean_acc|: ~9.81 for IMUs in g (Livox), ~1.0 for m/s^2 IMUs
        acc_scale_   = declare_parameter<double>("acc_scale", 9.81);
        double gyr_cov  = declare_parameter<double>("gyr_cov", 0.1);
        double acc_cov  = declare_parameter<double>("acc_cov", 0.1);
        double bgyr_cov = declare_parameter<double>("b_gyr_cov", 0.0001);
        double bacc_cov = declare_parameter<double>("b_acc_cov", 0.0001);
        Q_.setZero();
        Q_.diagonal() << V3::Constant(gyr_cov), V3::Constant(acc_cov),
                         V3::Constant(bgyr_cov), V3::Constant(bacc_cov);

        pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS());
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&HighRateOdom::imuCb, this, std::placeholders::_1));
        sub_state_ = create_subscription<fast_lio::msg::LioState>(
            state_topic_, 50,
            std::bind(&HighRateOdom::stateCb, this, std::placeholders::_1));
    }

private:
    // ---------------- IMU: propagate + publish at IMU rate ----------------
    void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        Imu s;
        s.t    = rclcpp::Time(msg->header.stamp).seconds() - time_offset_;
        s.gyro = V3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        s.acc  = V3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if (!buf_.empty() && s.t < buf_.back().t) {   // time jump back (bag loop)
            RCLCPP_WARN(get_logger(), "IMU time went backwards, resetting");
            buf_.clear();
            initialized_ = false;
        }
        buf_.push_back(s);
        while (buf_.size() > 2 && buf_.back().t - buf_.front().t > 10.0) buf_.pop_front();

        if (!initialized_) return;

        const Imu &head = buf_.size() >= 2 ? buf_[buf_.size() - 2] : s;
        step(head, s, std::max(head.t, t_state_), s.t);
        publish(s);
    }

    // ---------------- LIO state: reset + replay buffered IMU ----------------
    void stateCb(const fast_lio::msg::LioState::ConstSharedPtr m)
    {
        x_.pos  = V3(m->pos[0], m->pos[1], m->pos[2]);
        x_.vel  = V3(m->vel[0], m->vel[1], m->vel[2]);
        x_.bg   = V3(m->bg[0], m->bg[1], m->bg[2]);
        x_.ba   = V3(m->ba[0], m->ba[1], m->ba[2]);
        x_.grav = V3(m->grav[0], m->grav[1], m->grav[2]);
        x_.q    = Eigen::Quaterniond(m->rot[3], m->rot[0], m->rot[1], m->rot[2]).normalized();
        P_ = Eigen::Map<const Eigen::Matrix<double, 23, 23, Eigen::RowMajor>>(m->cov.data());
        t_state_ = rclcpp::Time(m->header.stamp).seconds();
        initialized_ = true;

        // drop IMU older than the last sample at/before the state stamp (kept as "head")
        while (buf_.size() >= 2 && buf_[1].t <= t_state_) buf_.pop_front();

        // replay everything newer than the state
        for (size_t k = 0; k < buf_.size(); ++k) {
            const Imu &tail = buf_[k];
            if (tail.t <= t_state_) continue;
            const Imu &head = (k > 0) ? buf_[k - 1] : tail;
            step(head, tail, std::max(head.t, t_state_), tail.t);
        }
        if (!buf_.empty() && buf_.back().t > t_state_) publish(buf_.back());
    }

    // ---------------- one mid-point propagation step (mirrors FAST-LIO) ----------------
    void step(const Imu &head, const Imu &tail, double t0, double t1)
    {
        const double dt = t1 - t0;
        if (dt <= 0.0) return;

        const V3 w = 0.5 * (head.gyro + tail.gyro) - x_.bg;
        const V3 a = 0.5 * (head.acc + tail.acc) * acc_scale_ - x_.ba;
        const M3 R = x_.q.toRotationMatrix();
        const V3 a_w = R * a + x_.grav;

        // covariance (error-state, same layout/noise model as FAST-LIO; gravity coupling neglected)
        M23 F = M23::Identity();
        F.block<3, 3>(0, 12)  = M3::Identity() * dt;
        F.block<3, 3>(3, 3)   = expSO3(-w * dt);
        F.block<3, 3>(3, 15)  = -M3::Identity() * dt;
        F.block<3, 3>(12, 3)  = -R * skew(a) * dt;
        F.block<3, 3>(12, 18) = -R * dt;

        M23x12 G = M23x12::Zero();
        G.block<3, 3>(3, 0)  = -M3::Identity() * dt;
        G.block<3, 3>(12, 3) = -R * dt;
        G.block<3, 3>(15, 6) = M3::Identity() * dt;
        G.block<3, 3>(18, 9) = M3::Identity() * dt;
        P_ = F * P_ * F.transpose() + G * Q_ * G.transpose();

        // state
        x_.pos += x_.vel * dt + 0.5 * a_w * dt * dt;
        x_.vel += a_w * dt;
        x_.q = Eigen::Quaterniond(R * expSO3(w * dt));
        x_.q.normalize();
        t_state_ = t1;
    }

    void publish(const Imu &s)
    {
        nav_msgs::msg::Odometry o;
        o.header.stamp = rclcpp::Time(static_cast<int64_t>(t_state_ * 1e9));
        o.header.frame_id = frame_id_;
        o.child_frame_id = child_id_;

        o.pose.pose.position.x = x_.pos.x();
        o.pose.pose.position.y = x_.pos.y();
        o.pose.pose.position.z = x_.pos.z();
        o.pose.pose.orientation.x = x_.q.x();
        o.pose.pose.orientation.y = x_.q.y();
        o.pose.pose.orientation.z = x_.q.z();
        o.pose.pose.orientation.w = x_.q.w();

        const M3 R = x_.q.toRotationMatrix();
        const V3 v_body = R.transpose() * x_.vel;
        const V3 w_body = s.gyro - x_.bg;
        o.twist.twist.linear.x = v_body.x();
        o.twist.twist.linear.y = v_body.y();
        o.twist.twist.linear.z = v_body.z();
        o.twist.twist.angular.x = w_body.x();
        o.twist.twist.angular.y = w_body.y();
        o.twist.twist.angular.z = w_body.z();

        // pose covariance in ROS order (x y z roll pitch yaw) from P: pos = 0..2, rot = 3..5
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                o.pose.covariance[i * 6 + j] = P_(i, j);

        const M3 Pv = R.transpose() * P_.block<3, 3>(12, 12) * R;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                o.twist.covariance[i * 6 + j] = Pv(i, j);
        const M3 Pbg = P_.block<3, 3>(15, 15);   // gyro-bias uncertainty as angular-rate uncertainty
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                o.twist.covariance[(i + 3) * 6 + (j + 3)] = Pbg(i, j);

        pub_->publish(o);
    }

    std::string imu_topic_, state_topic_, odom_topic_, frame_id_, child_id_;
    double time_offset_, acc_scale_;
    Eigen::Matrix<double, 12, 12> Q_;

    State x_;
    M23 P_ = M23::Identity();
    double t_state_ = 0.0;
    bool initialized_ = false;
    std::deque<Imu> buf_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<fast_lio::msg::LioState>::SharedPtr sub_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HighRateOdom>());   // single-threaded: callbacks never overlap
    rclcpp::shutdown();
    return 0;
}