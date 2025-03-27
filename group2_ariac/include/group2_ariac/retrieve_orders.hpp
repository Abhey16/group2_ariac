#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <queue>
#include <cstdint> 
#include "ariac_msgs/msg/order.hpp"

class Part
{
    public:
        // Default Constructor
        Part() = default;

        // Constructor
        Part(uint8_t input_color,uint8_t input_type)
        {
            color = input_color;
            type = input_type;
        }

        uint8_t get_color() const {return color;}
        uint8_t get_type() const {return type;}

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
        KittingPart( const Part& input_part,uint8_t input_quadrant)
        {   
            part = input_part;
            quadrant = input_quadrant;
        }

        const Part& get_part() const {return part;}
        uint8_t get_quadrant() const {return quadrant;}

    private:
        Part part;
        uint8_t quadrant{};

};

// Part class

// kitting class
class KittingTask
{
    public:
        // Default Constructor
        KittingTask() = default;

        // Constructor
        KittingTask(uint8_t input_agv_number,int8_t input_tray_id,uint8_t input_destination,const std::vector<KittingPart>& input_parts )
        {
            agv_number = input_agv_number;
            tray_id = input_tray_id;
            destination = input_destination;
            parts = input_parts;
        }

        u_int8_t get_agv_number() const {return agv_number;}
        int8_t get_tray_id() const {return tray_id;}
        uint8_t get_destination() const {return destination;}
        const std::vector<KittingPart>& get_parts() const {return parts;}

    private:
        uint8_t agv_number{};
        int8_t tray_id{};
        uint8_t destination{};
        std::vector<KittingPart> parts;

};

// assembly class -> Task Class

// combined class -> Task Class

// Task class


// order class
class Order
{
    public:
        //Default Constructor
        Order() = default;

        // Constructor
        Order(std::string input_id,uint8_t input_type,bool input_priority, const KittingTask& input_kitting_task)
        {
            id = input_id;
            type = input_type;
            priority = input_priority;
            kitting_task = input_kitting_task;

        }

        std::string get_id() const {return id;}
        u_int8_t get_type() const {return type;}
        bool get_priority() const {return priority;}
        const KittingTask& get_kitting_task() const {return kitting_task;}
        
    private:
        // fields
        std::string id{};
        uint8_t type{};
        bool priority{};
        KittingTask kitting_task;
};

class RetrieveOrders : public rclcpp::Node
{   
    public:
        // Constructor
        RetrieveOrders(std::string node_name) : Node(node_name)
        {
            subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders",10,
                                                                            std::bind(&RetrieveOrders::order_callback, this,
                                                                            std::placeholders::_1));

            // RCLCPP_INFO(this->get_logger(),"subscriber created");
        }

        // subscriber callback
        void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    private:
        // subscriber_
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_;

        // Storing Orders in a queue
        std::queue<Order> normal_orders;

        std::queue<Order> priority_orders;
};