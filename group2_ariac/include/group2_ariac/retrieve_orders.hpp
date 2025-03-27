#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <queue>
#include <cstdint>
#include "ariac_msgs/msg/order.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class Part
{
public:
    // Default Constructor
    Part() = default;

    // Constructor
    Part(uint8_t input_color, uint8_t input_type)
    {
        color = input_color;
        type = input_type;
    }

    uint8_t get_color() const { return color; }
    uint8_t get_type() const { return type; }

private:
    uint8_t color{};
    uint8_t type{};
};

// kitting Part -> Part
class KittingPart
{
public:
    // Default Constructor
    KittingPart() = default;

    // Constructor
    KittingPart(const Part &input_part, uint8_t input_quadrant)
    {
        part = input_part;
        quadrant = input_quadrant;
    }

    const Part &get_part() const { return part; }
    uint8_t get_quadrant() const { return quadrant; }

private:
    Part part;
    uint8_t quadrant{};
};

// Assembly Part
class AssemblyPart
{
public:
    AssemblyPart() = default;

    AssemblyPart(Part input_part,
                 geometry_msgs::msg::PoseStamped input_assembled_pose,
                 geometry_msgs::msg::Vector3 input_install_directions)
    {
        part = input_part;
        assembled_pose = input_assembled_pose;
        install_directions = input_install_directions;
    }

    const Part &get_part() const { return part; }
    geometry_msgs::msg::PoseStamped get_assembled_pose() const { return assembled_pose; }
    geometry_msgs::msg::Vector3 get_install_directions() const { return install_directions; }

private:
    Part part;
    geometry_msgs::msg::PoseStamped assembled_pose;
    geometry_msgs::msg::Vector3 install_directions;
};

// Part class

// kitting class
class KittingTask
{
public:
    // Default Constructor
    KittingTask() = default;

    // Constructor
    KittingTask(uint8_t input_agv_number,
                int8_t input_tray_id,
                uint8_t input_destination,
                const std::vector<KittingPart> &input_parts)
    {
        agv_number = input_agv_number;
        tray_id = input_tray_id;
        destination = input_destination;
        parts = input_parts;
    }

    u_int8_t get_agv_number() const { return agv_number; }
    int8_t get_tray_id() const { return tray_id; }
    uint8_t get_destination() const { return destination; }
    const std::vector<KittingPart> &get_parts() const { return parts; }

private:
    uint8_t agv_number{};
    int8_t tray_id{};
    uint8_t destination{};
    std::vector<KittingPart> parts;
};

// assembly class -> Task Class
class AssemblyTask
{
public:
    AssemblyTask() = default;

    AssemblyTask(std::vector<u_int8_t> input_agv_numbers,
                 u_int8_t input_station,
                 std::vector<AssemblyPart> input_parts)
    {
        agv_numbers = input_agv_numbers;
        station = input_station;
        parts = input_parts;
    }

    const std::vector<u_int8_t> &get_agv_numbers() const { return agv_numbers; }
    u_int8_t get_station() const { return station; }
    const std::vector<AssemblyPart> &get_parts() const { return parts; }

private:
    std::vector<u_int8_t> agv_numbers;
    u_int8_t station;
    std::vector<AssemblyPart> parts;
};

// combined class -> Task Class
class CombinedTask
{
public:
    CombinedTask() = default;

    CombinedTask(u_int8_t input_station, std::vector<AssemblyPart> input_parts)
    {
        station = input_station;
        parts = input_parts;
    }

    u_int8_t get_station() const { return station; }
    const std::vector<AssemblyPart> &get_parts() const { return parts; }

private:
    u_int8_t station;
    std::vector<AssemblyPart> parts;
};

// order class
class Order
{
public:
    // Default Constructor
    Order() = default;

    // Constructor
    Order(std::string input_id,
          uint8_t input_type,
          bool input_priority,
          const KittingTask &input_kitting_task,
          const AssemblyTask &input_assembly_task,
          const CombinedTask &input_combined_task)
    {
        id = input_id;
        type = input_type;
        priority = input_priority;
        kitting_task = input_kitting_task;
        assembly_task = input_assembly_task;
        combined_task = input_combined_task;
    }

    std::string get_id() const { return id; }
    u_int8_t get_type() const { return type; }
    bool get_priority() const { return priority; }
    const KittingTask &get_kitting_task() const { return kitting_task; }
    const AssemblyTask &get_assembly_task() const { return assembly_task; }
    const CombinedTask &get_combined_task() const { return combined_task; }

private:
    // fields
    std::string id{};
    uint8_t type{};
    bool priority{};
    KittingTask kitting_task;
    AssemblyTask assembly_task;
    CombinedTask combined_task;
};

class RetrieveOrders : public rclcpp::Node
{
public:
    // Constructor
    RetrieveOrders(std::string node_name) : Node(node_name)
    {
        subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10,
                                                                        std::bind(&RetrieveOrders::order_callback, this,
                                                                                  std::placeholders::_1));

        // RCLCPP_INFO(this->get_logger(),"subscriber created");
    }

    // subscriber callback
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    void display_order(const Order& order);

private:
    // subscriber_
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_;

    // Storing Orders in a queue
    std::queue<Order> normal_orders;

    std::queue<Order> priority_orders;
};