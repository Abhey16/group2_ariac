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
        color = set_color(input_color);
        type = set_type(input_type);
    }

    // set color
    std::string set_color(uint8_t input_color)
    {
        switch (input_color)
        {
        case ariac_msgs::msg::Part::RED:
            return "RED";
        case ariac_msgs::msg::Part::GREEN:
            return "GREEN";
        case ariac_msgs::msg::Part::BLUE:
            return "BLUE";
        case ariac_msgs::msg::Part::ORANGE:
            return "ORANGE";
        case ariac_msgs::msg::Part::PURPLE:
            return "PURPLE";
        default:
            return "no type added";
        }
    }

    // set Part Type
    std::string set_type(uint8_t input_type)
    {
        switch (input_type)
        {
        case ariac_msgs::msg::Part::BATTERY:
            return "BATTERY";
        case ariac_msgs::msg::Part::PUMP:
            return "PUMP";
        case ariac_msgs::msg::Part::SENSOR:
            return "SENSOR";
        case ariac_msgs::msg::Part::REGULATOR:
            return "REGULATOR";
        default:
            return "no type added";
        }
    }

    // getter function
    std::string get_color() const { return color; }
    std::string get_type() const { return type; }

private:
    std::string color{};
    std::string type{};
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
        quadrant = set_quadrant(input_quadrant);
    }

    // set quadrant
    std::string set_quadrant(uint8_t input_quadrant)
    {
        switch (input_quadrant)
        {
        case ariac_msgs::msg::KittingPart::QUADRANT1:
            return "QUADRANT1";
        case ariac_msgs::msg::KittingPart::QUADRANT2:
            return "QUADRANT2";
        case ariac_msgs::msg::KittingPart::QUADRANT3:
            return "QUADRANT3";
        case ariac_msgs::msg::KittingPart::QUADRANT4:
            return "QUADRANT4";
        default:
            return "no type added";
        }
    }

    // getter function
    const Part &get_part() const { return part; }
    std::string get_quadrant() const { return quadrant; }

private:
    Part part;
    std::string quadrant;
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
        destination = set_destination(input_destination);
        parts = input_parts;
    }

    // set destination
    std::string set_destination(uint8_t input_destination)
    {
        switch (input_destination)
        {
        case ariac_msgs::msg::KittingTask::KITTING:
            return "KITTING";
        case ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT:
            return "ASSEMBLY_FRONT";
        case ariac_msgs::msg::KittingTask::ASSEMBLY_BACK:
            return "ASSEMBLY_BACK";
        case ariac_msgs::msg::KittingTask::WAREHOUSE:
            return "WAREHOUSE";
        default:
            return "no type added";
        }
    }

    // getter function
    u_int8_t get_agv_number() const { return agv_number; }
    int8_t get_tray_id() const { return tray_id; }
    std::string get_destination() const { return destination; }
    const std::vector<KittingPart> &get_parts() const { return parts; }

private:
    uint8_t agv_number{};
    int8_t tray_id{};
    std::string destination{};
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
        station = set_station(input_station);
        parts = input_parts;
    }

    // set assembly station
    std::string set_station(uint8_t input_station)
    {
        switch (input_station)
        {
        case ariac_msgs::msg::AssemblyTask::AS1:
            return "AS1";
        case ariac_msgs::msg::AssemblyTask::AS2:
            return "AS2";
        case ariac_msgs::msg::AssemblyTask::AS3:
            return "AS3";
        case ariac_msgs::msg::AssemblyTask::AS4:
            return "AS4";
        default:
            return "no type added";
        }
    }

    // getter function
    const std::vector<u_int8_t> &get_agv_numbers() const { return agv_numbers; }
    std::string get_station() const { return station; }
    const std::vector<AssemblyPart> &get_parts() const { return parts; }

private:
    std::vector<u_int8_t> agv_numbers;
    std::string station;
    std::vector<AssemblyPart> parts;
};

// combined class -> Task Class
class CombinedTask
{
public:
    CombinedTask() = default;

    CombinedTask(u_int8_t input_station, std::vector<AssemblyPart> input_parts)
    {
        station = set_station(input_station);
        parts = input_parts;
    }

    // set assembly station
    std::string set_station(uint8_t input_station)
    {
        switch (input_station)
        {
        case ariac_msgs::msg::AssemblyTask::AS1:
            return "AS1";
        case ariac_msgs::msg::AssemblyTask::AS2:
            return "AS2";
        case ariac_msgs::msg::AssemblyTask::AS3:
            return "AS3";
        case ariac_msgs::msg::AssemblyTask::AS4:
            return "AS4";
        default:
            return "no type added";
        }
    }

    // getter function
    std::string get_station() const { return station; }
    const std::vector<AssemblyPart> &get_parts() const { return parts; }

private:
    std::string station;
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
        type = set_type(input_type);
        priority = input_priority;
        kitting_task = input_kitting_task;
        assembly_task = input_assembly_task;
        combined_task = input_combined_task;
    }

    // set type of task
    std::string set_type(uint8_t input_type)
    {
        switch (input_type)
        {
        case ariac_msgs::msg::Order::KITTING:
            return "KITTING";
        case ariac_msgs::msg::Order::ASSEMBLY:
            return "ASSEMBLY";
        case ariac_msgs::msg::Order::COMBINED:
            return "COMBINED";
        default:
            return "no type added";
        }
    }

    // getter functions
    std::string get_id() const { return id; }
    std::string get_type() const { return type; }
    bool get_priority() const { return priority; }
    const KittingTask &get_kitting_task() const { return kitting_task; }
    const AssemblyTask &get_assembly_task() const { return assembly_task; }
    const CombinedTask &get_combined_task() const { return combined_task; }

private:
    // fields
    std::string id{};
    std::string type{};
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
    void display_order(const Order &order);

private:
    // subscriber_
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_;

    // Storing Orders in a queue
    std::queue<Order> normal_orders;

    std::queue<Order> priority_orders;
};