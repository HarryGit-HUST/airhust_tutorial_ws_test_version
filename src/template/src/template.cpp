#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h> // 新增：修复height_error发布依赖
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <iostream>

using namespace std;

// ===================== 全局变量定义 =====================
int mission_num = 0;
float if_debug = 1.0; // 仿真模式：自动OFFBOARD
float err_max = 0.1;
float ALTITUDE = 1.5f; // 目标高度（强制锁定）
float final_target_x = 17.0;
float final_target_y = -0.3;
float task3_target_x = 36.0;
float task3_target_y = 1.0;

// 激光雷达相关
sensor_msgs::LaserScan laser_data;
int center_idx = 0;
int half_idx = 0;
float laser_angle_increment = 0.0;
float obstacle_dist = 100.0;
bool is_avoiding = false;
bool is_takeoff_stable = false; // 仅起飞阶段判断（永久有效）
ros::Time takeoff_stable_time;
bool is_obstacle_detected = false;

// 避障核心参数（优化绕障逻辑）
const float SAFE_DIST = 1.7;           // 安全距离（易触发绕障）
const float COLLISION_DIST = 1.1;      // 紧急距离
const int SCAN_ANGLE = 50;             // 检测角度±50度（覆盖柱子两侧）
const float MAX_FORWARD_SPEED = 0.9;   // 低速平稳，便于绕障
const float MAX_SIDE_SPEED = 1.0;      // 降低横向速度，避免左右晃
const float BASE_AVOID_OFFSET = 1.5;   // 基础偏移量（平衡绕障与横移）
const float OBSTACLE_VALID_DIST = 0.6; // 低过滤阈值，确保柱子检测
const float FINAL_DIST_THRESHOLD = 0.5;
const float MAX_YAW_OFFSET = 3.5;     // 适度扩大横移限制（便于绕柱）
const int OBSTACLE_CONFIRM_COUNT = 2; // 连续2帧确认（快速触发）
int obstacle_confirm_cnt = 0;
const float OBSTACLE_PASS_THRESHOLD = 1.2; // 柱子通过判定阈值
bool is_obstacle_passed = false;           // 避免重复绕障

// 高度控制参数（强制稳定）
const float HEIGHT_PID_KP = 1.2;     // 高度比例系数（强化控制）
const float HEIGHT_ERROR_MAX = 0.08; // 高度误差允许范围

// 头文件声明的全局变量
mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::State current_state;
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;

// 可视化发布器（修复语法错误）
ros::Publisher target_marker_pub;
ros::Publisher obstacle_status_pub;
ros::Publisher height_error_pub; // 高度误差发布器（已修复）

// ===================== 工具函数（高度PID强制控制）=====================
float height_pid_control(float current_z, float target_z)
{
    float error = target_z - current_z;
    // 发布高度误差
    std_msgs::Float32 height_error_msg;
    height_error_msg.data = error;
    height_error_pub.publish(height_error_msg);

    // 误差在允许范围，直接返回目标高度
    if (fabs(error) < HEIGHT_ERROR_MAX)
    {
        return target_z;
    }

    // PID比例控制（修复：将限幅参数改为float类型，添加f后缀）
    float output_z = current_z + error * HEIGHT_PID_KP;
    // 关键修复：0.1改为0.1f，确保与output_z（float）类型一致
    output_z = std::clamp(output_z, target_z - 0.1f, target_z + 0.1f);
    return output_z;
}

// ===================== 回调函数实现 =====================
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 初始化起飞点
    if (!flag_init_position && local_pos.pose.pose.position.z > 0.5)
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
        ROS_INFO("【起飞点】初始化完成：X=%.2f, Y=%.2f, Z=%.2f",
                 init_position_x_take_off, init_position_y_take_off, init_position_z_take_off);
    }

    // 起飞稳定判定（仅执行一次，永久有效）
    if (flag_init_position && !is_takeoff_stable)
    {
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.15)
        {
            if (!takeoff_stable_time.isValid())
            {
                takeoff_stable_time = ros::Time::now();
                ROS_INFO("【起飞稳定】开始计时...");
            }
            else if ((ros::Time::now() - takeoff_stable_time) > ros::Duration(1.0))
            {
                is_takeoff_stable = true;
                ROS_INFO("【起飞稳定】完成！永久开启避障，强制锁定高度=%.2f米", ALTITUDE);
            }
        }
        else
        {
            takeoff_stable_time = ros::Time::now();
            ROS_WARN("【起飞稳定】高度不稳定（当前%.2f米，目标%.2f米），重置计时",
                     local_pos.pose.pose.position.z, ALTITUDE);
        }
    }

    // 高度状态日志（便于调试）
    ROS_INFO_THROTTLE(0.5, "【高度状态】当前=%.2f米 | 目标=%.2f米 | 误差=%.3f米",
                      local_pos.pose.pose.position.z, ALTITUDE,
                      fabs(local_pos.pose.pose.position.z - ALTITUDE));
}

// 激光雷达回调（优化柱子检测）
void laser_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_data = *msg;
    laser_angle_increment = laser_data.angle_increment;

    if (laser_data.ranges.empty())
    {
        ROS_ERROR("【激光雷达】无有效数据！");
        obstacle_dist = 100.0;
        obstacle_confirm_cnt = 0;
        is_obstacle_detected = false;
        return;
    }

    // 计算正前方及检测范围索引
    float center_angle = 0.0;
    center_idx = round((center_angle - laser_data.angle_min) / laser_angle_increment);
    center_idx = std::clamp(center_idx, 0, (int)laser_data.ranges.size() - 1);

    float half_angle = SCAN_ANGLE * M_PI / 180.0;
    half_idx = round(half_angle / laser_angle_increment);
    half_idx = min(half_idx, (int)laser_data.ranges.size() / 2);
    int start_idx = max(0, center_idx - half_idx);
    int end_idx = min((int)laser_data.ranges.size() - 1, center_idx + half_idx);

    // 障碍物检测（重点关注正前方柱子）
    obstacle_dist = 100.0;
    int valid_point_cnt = 0;
    int front_start = max(0, center_idx - half_idx / 3); // 正前方核心区域
    int front_end = min((int)laser_data.ranges.size() - 1, center_idx + half_idx / 3);

    for (int i = start_idx; i <= end_idx; ++i)
    {
        float dist = laser_data.ranges[i];
        if (dist > OBSTACLE_VALID_DIST && dist < 6.0 && !isinf(dist) && !isnan(dist))
        {
            obstacle_dist = min(obstacle_dist, dist);
            if (i >= front_start && i <= front_end)
            {
                valid_point_cnt++; // 正前方有效点计数（确认柱子）
            }
        }
    }

    // 障碍物确认（连续2帧+正前方有效点>3个）
    bool current_obstacle = (is_takeoff_stable && obstacle_dist < SAFE_DIST && valid_point_cnt > 3);
    if (current_obstacle)
    {
        obstacle_confirm_cnt++;
        if (obstacle_confirm_cnt >= OBSTACLE_CONFIRM_COUNT)
        {
            is_obstacle_detected = true;
            is_obstacle_passed = false; // 重置通过标志
        }
    }
    else
    {
        obstacle_confirm_cnt = 0;
        // 柱子通过判定：障碍物消失+横向回归
        if (is_avoiding && fabs(local_pos.pose.pose.position.y - (final_target_y + init_position_y_take_off)) < 0.3)
        {
            is_obstacle_passed = true;
            is_obstacle_detected = false;
        }
    }

    // 发布障碍物状态
    std_msgs::Bool obstacle_status;
    obstacle_status.data = is_obstacle_detected;
    obstacle_status_pub.publish(obstacle_status);

    // 调试日志（突出柱子检测）
    ROS_INFO_THROTTLE(0.3,
                      "【激光雷达】柱子距离=%.2f米 | 正前方有效点数=%d | 确认次数=%d/%d | 已通过=%s | 可避障=%s",
                      obstacle_dist, valid_point_cnt,
                      obstacle_confirm_cnt, (int)OBSTACLE_CONFIRM_COUNT,
                      is_obstacle_passed ? "是" : "否",
                      is_obstacle_detected ? "是" : "否");
}

// 精确降落函数
bool precision_land()
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
        ROS_INFO("【降落】开始：X=%.2f, Y=%.2f", precision_land_init_position_x, precision_land_init_position_y);
    }

    static float current_land_z = ALTITUDE;
    current_land_z = std::max(0.0f, current_land_z - 0.08f);

    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = current_land_z;
    setpoint_raw.yaw = init_yaw_take_off;
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
    setpoint_raw.coordinate_frame = 1;

    if (current_land_z <= 0.1 && (ros::Time::now() - precision_land_last_time) > ros::Duration(1.0))
    {
        ROS_INFO("【降落】完成！高度：%.2f米", current_land_z);
        precision_land_init_position_flag = false;
        current_land_z = ALTITUDE;
        return true;
    }
    return false;
}

// ===================== 巡航+避障核心函数（最终优化）=====================
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (!mission_pos_cruise_flag)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        ROS_INFO("【巡航】启动：目标（相对起飞点）X=%.2f, Y=%.2f, 高度=%.2f", x, y, z);
    }

    // 世界系目标位置
    float target_x_world = x + init_position_x_take_off;
    float target_y_world = y + init_position_y_take_off;
    float target_z_world = z + init_position_z_take_off;

    // 强制高度控制（核心解决未达高度问题）
    target_z_world = height_pid_control(local_pos.pose.pose.position.z, ALTITUDE);

    // 速度控制（低速平稳，便于绕障）
    float dx = target_x_world - local_pos.pose.pose.position.x;
    float x_speed = dx * 0.6; // 降低比例系数，避免超调
    x_speed = std::clamp(x_speed, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);

    // ===================== 避障逻辑（解决柱子左右晃）=====================
    float adjusted_y = target_y_world;
    static float current_avoid_offset = 0.0;
    static ros::Time obstacle_avoid_start_time;

    // 避障触发：检测到柱子且未通过
    if (is_obstacle_detected && !is_obstacle_passed)
    {
        if (!is_avoiding)
        {
            ROS_INFO("【避障】检测到正对柱子，开始绕障！距离=%.2f米", obstacle_dist);
            is_avoiding = true;
            obstacle_avoid_start_time = ros::Time::now();

            // 一次性判断绕障方向（不切换，避免左右晃）
            float left_avg = 0.0, right_avg = 0.0;
            int left_cnt = 0, right_cnt = 0;
            int left_start = max(0, center_idx - 2 * half_idx);
            int left_end = max(0, center_idx - half_idx / 2);
            int right_start = min((int)laser_data.ranges.size() - 1, center_idx + half_idx / 2);
            int right_end = min((int)laser_data.ranges.size() - 1, center_idx + 2 * half_idx);

            // 统计左右侧距离
            for (int i = left_start; i <= left_end; ++i)
            {
                float dist = laser_data.ranges[i];
                if (dist > OBSTACLE_VALID_DIST && dist < 6.0)
                    left_avg += dist, left_cnt++;
            }
            for (int i = right_start; i <= right_end; ++i)
            {
                float dist = laser_data.ranges[i];
                if (dist > OBSTACLE_VALID_DIST && dist < 6.0)
                    right_avg += dist, right_cnt++;
            }

            left_avg = left_cnt > 0 ? left_avg / left_cnt : SAFE_DIST;
            right_avg = right_cnt > 0 ? right_avg / right_cnt : SAFE_DIST;

            // 选择更安全的绕障方向（一旦确定，全程不变）
            current_avoid_offset = (left_avg > right_avg) ? BASE_AVOID_OFFSET : -BASE_AVOID_OFFSET;
            ROS_INFO("【避障】固定绕障方向：%s（左距=%.2f，右距=%.2f）",
                     current_avoid_offset > 0 ? "向左" : "向右", left_avg, right_avg);
        }

        // 动态减速（柱子越近，速度越慢）
        float decelerate_ratio = obstacle_dist < COLLISION_DIST ? 0.2 : 0.3;
        x_speed *= decelerate_ratio;

        // 平滑横移，避免突变
        float target_y_avoid = target_y_world + current_avoid_offset;
        float dy = target_y_avoid - local_pos.pose.pose.position.y;
        dy = std::clamp(dy, -MAX_SIDE_SPEED / 2, MAX_SIDE_SPEED / 2); // 横向速度减半
        adjusted_y = local_pos.pose.pose.position.y + dy;

        // 柱子通过判定：正前方无近距离障碍物
        float front_obstacle_dist = 100.0;
        int front_start = max(0, center_idx - half_idx / 4);
        int front_end = min((int)laser_data.ranges.size() - 1, center_idx + half_idx / 4);
        for (int i = front_start; i <= front_end; ++i)
        {
            float dist = laser_data.ranges[i];
            if (dist > OBSTACLE_VALID_DIST && dist < 6.0)
            {
                front_obstacle_dist = min(front_obstacle_dist, dist);
            }
        }
        if (front_obstacle_dist > OBSTACLE_PASS_THRESHOLD ||
            (ros::Time::now() - obstacle_avoid_start_time) > ros::Duration(5.0))
        {
            is_obstacle_passed = true;
            ROS_INFO("【避障】柱子通过！正前方距离=%.2f米，开始回归", front_obstacle_dist);
        }

        ROS_INFO_THROTTLE(0.2, "【避障】方向：%s | 偏移=%.2f | 目标Y=%.2f | X速度=%.2f",
                          current_avoid_offset > 0 ? "向左" : "向右", current_avoid_offset, adjusted_y, x_speed);
    }
    // 避障退出：缓慢回归原路径
    else if (is_avoiding)
    {
        float dy = target_y_world - local_pos.pose.pose.position.y;
        dy = std::clamp(dy, -MAX_SIDE_SPEED / 3, MAX_SIDE_SPEED / 3); // 超低速回归
        adjusted_y = local_pos.pose.pose.position.y + dy;

        if (fabs(local_pos.pose.pose.position.y - target_y_world) < 0.2)
        {
            is_avoiding = false;
            current_avoid_offset = 0.0;
            ROS_INFO("【避障】回归原路径完成！");
        }
    }
    // 无避障：正常跟踪
    else
    {
        float dy = target_y_world - local_pos.pose.pose.position.y;
        dy = std::clamp(dy, -MAX_SIDE_SPEED, MAX_SIDE_SPEED);
        adjusted_y = local_pos.pose.pose.position.y + dy;
    }

    // 目标位置更新
    target_x_world = local_pos.pose.pose.position.x + x_speed;
    target_y_world = adjusted_y;

    // 控制指令设置（强制高度）
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = target_x_world;
    setpoint_raw.position.y = adjusted_y;
    setpoint_raw.position.z = target_z_world;
    setpoint_raw.yaw = target_yaw;

    // RViz可视化（高度误差颜色提示）
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "map";
    target_marker.header.stamp = ros::Time::now();
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.pose.position.x = target_x_world;
    target_marker.pose.position.y = target_y_world;
    target_marker.pose.position.z = target_z_world;
    target_marker.scale.x = 0.3;
    target_marker.scale.y = 0.3;
    target_marker.scale.z = 0.3;

    // 高度误差颜色：绿（<0.08m）→黄（<0.16m）→红（≥0.16m）
    float height_error = fabs(local_pos.pose.pose.position.z - target_z_world);
    if (height_error < HEIGHT_ERROR_MAX)
    {
        target_marker.color.r = is_avoiding ? 0.0f : 0.0f;
        target_marker.color.g = 1.0f;
    }
    else if (height_error < HEIGHT_ERROR_MAX * 2)
    {
        target_marker.color.r = 1.0f;
        target_marker.color.g = 1.0f;
    }
    else
    {
        target_marker.color.r = 1.0f;
        target_marker.color.g = 0.0f;
    }
    target_marker.color.b = 0.0f;
    target_marker.color.a = 1.0f;
    target_marker_pub.publish(target_marker);

    // 临时目标到达判断
    bool x_ok = fabs(local_pos.pose.pose.position.x - target_x_world) < error_max;
    bool y_ok = fabs(local_pos.pose.pose.position.y - adjusted_y) < error_max;
    bool z_ok = fabs(local_pos.pose.pose.position.z - target_z_world) < 0.1; // 高度误差放宽
    bool yaw_ok = fabs(yaw - target_yaw) < 0.15;

    ROS_INFO_THROTTLE(0.3, "[巡航状态] 当前(%.2f,%.2f,%.2f) → 目标(%.2f,%.2f,%.2f) | 避障：%s",
                      local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z,
                      target_x_world, adjusted_y, target_z_world,
                      is_avoiding ? "开启（绕柱）" : "关闭");

    return (x_ok && y_ok && z_ok && yaw_ok);
}

// ===================== 主函数（修复发布器语法）=====================
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "drone_obstacle_avoidance_final");
    ros::NodeHandle nh;

    // 订阅/发布话题（修复height_error_pub语法）
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, laser_cb);
    ros::Publisher mavros_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    target_marker_pub = nh.advertise<visualization_msgs::Marker>("drone_target_marker", 10);
    obstacle_status_pub = nh.advertise<std_msgs::Bool>("obstacle_detected", 10);
    // 修复：明确指定消息类型为std_msgs::Float32，语法正确
    height_error_pub = nh.advertise<std_msgs::Float32>("height_error", 10);

    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);

    // 读取参数
    nh.param<float>("ALTITUDE", ALTITUDE, 1.5);
    nh.param<float>("err_max", err_max, 0.1);
    nh.param<float>("if_debug", if_debug, 1.0);
    nh.param<float>("final_target_x", final_target_x, 8.0);
    nh.param<float>("final_target_y", final_target_y, 0.0);
    nh.param<float>("task3_target_x", task3_target_x, 35.0);
    nh.param<float>("task3_target_y", task3_target_y, 1.0);

    // 打印配置
    ROS_INFO("=== 无人机避障巡航（最终修复版）===");
    ROS_INFO("核心功能：");
    ROS_INFO("  1. 起飞稳定后永久开启避障（仅起飞判断高度）");
    ROS_INFO("  2. PID强制高度锁定（目标=%.2f米，误差≤0.1m）", ALTITUDE);
    ROS_INFO("  3. 正对柱子：固定绕障方向，无左右晃，顺畅通过");
    ROS_INFO("避障参数：安全距离=%.2f米，横移限制=%.2f米，横向速度=%.2f m/s",
             SAFE_DIST, MAX_YAW_OFFSET, MAX_SIDE_SPEED);
    ROS_INFO("自动启动任务...");

    // 等待飞控连接
    ROS_INFO("等待飞控连接...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控连接成功！");

    // 发送初始目标点（强制高度）
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    ROS_INFO("发送初始目标点（100个），强制高度=%.2f米...", ALTITUDE);
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("初始目标点发送完成！");

    // 自动OFFBOARD+解锁
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ROS_INFO("自动启用OFFBOARD+解锁...");
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(3.0))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("OFFBOARD模式已启用！");
            }
            else
            {
                ROS_WARN("OFFBOARD模式切换失败，3秒后重试...");
            }
            last_request = ros::Time::now();
        }
        else if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(3.0))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("无人机已解锁！");
            }
            else
            {
                ROS_WARN("解锁失败，3秒后重试...");
            }
            last_request = ros::Time::now();
        }

        // 起飞悬停1秒（确保高度稳定）
        if (current_state.armed && current_state.mode == "OFFBOARD" &&
            fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
        {
            if ((ros::Time::now() - last_request) > ros::Duration(1.0))
            {
                mission_num = 1;
                last_request = ros::Time::now();
                ROS_INFO("起飞完成，进入任务1（悬停稳定）！");
                break;
            }
        }

        // 强制发布高度控制指令
        setpoint_raw.position.z = ALTITUDE;
        mavros_setpoint_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // 执行任务流程
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "=== 当前任务：%d ===", mission_num);

        switch (mission_num)
        {
        // 任务1：悬停稳定（强制高度）
        case 1:
            if (mission_pos_cruise(0, 0, ALTITUDE, 0.0, err_max) ||
                (ros::Time::now() - last_request) > ros::Duration(3.0))
            {
                ROS_INFO("任务1完成：悬停稳定！当前高度=%.2f米", local_pos.pose.pose.position.z);
                mission_num = 2;
                mission_pos_cruise_flag = false;
                last_request = ros::Time::now();
            }
            break;

        // 任务2：巡航至第一个目标点（绕柱优化）
        case 2:
        {
            mission_pos_cruise(final_target_x, final_target_y, ALTITUDE, 0.0, err_max);
            float final_x_world = final_target_x + init_position_x_take_off;
            float final_y_world = final_target_y + init_position_y_take_off;
            float dist_to_final = sqrt(pow(local_pos.pose.pose.position.x - final_x_world, 2) +
                                       pow(local_pos.pose.pose.position.y - final_y_world, 2));

            if (dist_to_final < FINAL_DIST_THRESHOLD)
            {
                ROS_INFO("任务2完成：到达第一个目标点！误差=%.2f米，高度=%.2f米",
                         dist_to_final, local_pos.pose.pose.position.z);
                mission_num = 3;
                mission_pos_cruise_flag = false;
                last_request = ros::Time::now();
            }
            break;
        }

        // 任务3：巡航至第二个目标点（绕柱优化）
        case 3:
        {
            mission_pos_cruise(task3_target_x, task3_target_y, ALTITUDE, 0.0, err_max);
            float final_x_world = task3_target_x + init_position_x_take_off;
            float final_y_world = task3_target_y + init_position_y_take_off;
            float dist_to_final = sqrt(pow(local_pos.pose.pose.position.x - final_x_world, 2) +
                                       pow(local_pos.pose.pose.position.y - final_y_world, 2));

            if (dist_to_final < FINAL_DIST_THRESHOLD)
            {
                ROS_INFO("任务3完成：到达第二个目标点！误差=%.2f米，高度=%.2f米",
                         dist_to_final, local_pos.pose.pose.position.z);
                mission_num = 4;
                last_request = ros::Time::now();
            }
            break;
        }

        // 任务4：降落
        case 4:
            if (precision_land())
            {
                ROS_INFO("任务4完成：降落成功！");
                mission_num = -1;
                last_request = ros::Time::now();
            }
            break;

        // 任务结束
        case -1:
            ROS_INFO("所有任务完成，程序退出！");
            return 0;
            break;

        default:
            ROS_WARN("未知任务编号：%d", mission_num);
            break;
        }

        mavros_setpoint_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
