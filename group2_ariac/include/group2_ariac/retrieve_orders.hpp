/**
 * @file retrieve_orders.hpp
 * @brief Defines classes for managing parts, kitting, assembly, and order retrieval in an ROS 2 environment.
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <queue>
#include <cstdint>
#include "ariac_msgs/msg/order.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "ariac_msgs/msg/advanced_logical_camera_image.hpp"
#include "ariac_msgs/msg/kit_tray_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
 
 /**
  * @class Part
  * @brief Represents a part with a specific color and type.
  */
class Part
{
public:
     /**
      * @brief Default constructor.
      */
    Part() = default;

     /**
      * @brief Parameterized constructor.
      * @param input_color The color of the part.
      * @param input_type The type of the part.
      */
    Part(uint8_t input_color, uint8_t input_type)
    {
        color = set_color(input_color);
        type = set_type(input_type);
    }

     /**
      * @brief Sets the color of the part.
      * @param input_color The color code.
      * @return The color as a string.
      */
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

     /**
      * @brief Sets the type of the part.
      * @param input_type The type code.
      * @return The type as a string.
      */
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

     /**
      * @brief Gets the color of the part.
      * @return The color as a string.
      */
    std::string get_color() const { return color; }

    /**
     * @brief Gets the type of the part.
     * @return The type as a string.
     */
    std::string get_type() const { return type; }

private:
    std::string color{}; ///< The color of the part.
    std::string type{}; ///< The type of the part.
};

 /**
  * @class KittingPart
  * @brief Represents a kitting part, which consists of a part and a quadrant.
  */
class KittingPart
{
public:
    /**
     * @brief Default constructor.
     */
    KittingPart() = default;

    /**
     * @brief Parameterized constructor.
     * @param input_part Part object
     * @param input_quadrant The quadrant
     */
    KittingPart(const Part &input_part, uint8_t input_quadrant)
    {
        part = input_part;
        quadrant = set_quadrant(input_quadrant);
    }

    /**
     * @brief Sets the quadrant of the part.
     * @param input_quadrant.
     * @return The quadrant as a string.
     */
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

    /**
     * @brief Gets the Part object.
     * @return Part object
     */
    const Part &get_part() const { return part; }
    /**
     * @brief Gets the quadrant of the part.
     * @return The quadrant of part as a string.
     */   
    std::string get_quadrant() const { return quadrant; }

private:
    Part part; ///< The part associated with kitting.
    std::string quadrant; ///< The quadrant in which the part is placed.
};

 /**
  * @class AssemblyPart
  * @brief Represents an assembly part, which consists of a part, assembled pose, and installation directions.
  */
class AssemblyPart
{
public:
    /**
     * @brief Default constructor.
     */
    AssemblyPart() = default;

    /**
     * @brief Parameterized constructor.
     * @param input_part Part object
     * @param input_assembled_pose The assembly pose
     * @param input_install_directions The installatin directions
     */
    AssemblyPart(Part input_part,
                 geometry_msgs::msg::PoseStamped input_assembled_pose,
                 geometry_msgs::msg::Vector3 input_install_directions)
    {
        part = input_part;
        assembled_pose = input_assembled_pose;
        install_directions = input_install_directions;
    }
    /**
     * @brief Gets the Part object.
     * @return Part object
     */
    const Part &get_part() const { return part; }
    /**
     * @brief Gets the Pose of the part
     * @return Pose of the part
     */
    geometry_msgs::msg::PoseStamped get_assembled_pose() const { return assembled_pose; }
    /**
     * @brief Gets the installation directions
     * @return installation direction
     */
    geometry_msgs::msg::Vector3 get_install_directions() const { return install_directions; }

private:
    Part part; ///< The part to be assembled.
    geometry_msgs::msg::PoseStamped assembled_pose; ///< The pose where the part is assembled.
    geometry_msgs::msg::Vector3 install_directions; ///< The installation direction.
};

/**
 * @class KittingTask
 * @brief Represents a kitting task.
 */
class KittingTask
{
public:
    /** @brief Default constructor. */
    KittingTask() = default;

    /**
     * @brief Constructor to initialize a kitting task.
     * @param input_agv_number The AGV number assigned to the task.
     * @param input_tray_id The tray ID used in the task.
     * @param input_destination The destination of the kitting task.
     * @param input_parts List of parts involved in the task.
     */
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

    /**
     * @brief Sets the destination of the kitting task.
     * @param input_destination The destination identifier.
     * @return The destination as a string.
     */
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

    /** @brief Gets the AGV number assigned to the task. */
    u_int8_t get_agv_number() const { return agv_number; }
    /** @brief Gets the tray ID used in the task. */
    int8_t get_tray_id() const { return tray_id; }
    /** @brief Gets the destination of the kitting task. */
    std::string get_destination() const { return destination; }
    /** @brief Gets the list of parts involved in the task. */
    const std::vector<KittingPart> &get_parts() const { return parts; }

private:
    uint8_t agv_number{};
    int8_t tray_id{};
    std::string destination{};
    std::vector<KittingPart> parts;
};

/**
 * @class AssemblyTask
 * @brief Represents an assembly task.
 */
class AssemblyTask
{
public:
    /** @brief Default constructor. */
    AssemblyTask() = default;
    /**
     * @brief Constructor to initialize an assembly task.
     * @param input_agv_numbers List of AGV numbers assigned to the task.
     * @param input_station The assembly station.
     * @param input_parts List of parts involved in the task.
     */
    AssemblyTask(std::vector<u_int8_t> input_agv_numbers,
                 u_int8_t input_station,
                 std::vector<AssemblyPart> input_parts)
    {
        agv_numbers = input_agv_numbers;
        station = set_station(input_station);
        parts = input_parts;
    }

    /**
     * @brief Sets the assembly station.
     * @param input_station The station identifier.
     * @return The station as a string.
     */
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

    /** @brief Gets the list of AGV numbers assigned to the task. */
    const std::vector<u_int8_t> &get_agv_numbers() const { return agv_numbers; }

    /** @brief Gets the station of the assembly task. */   
    std::string get_station() const { return station; }

    /** @brief Gets the list of parts involved in the task. */  
    const std::vector<AssemblyPart> &get_parts() const { return parts; }

private:
    std::vector<u_int8_t> agv_numbers; ///< List of AGV numbers
    std::string station; ///< Assembly station 
    std::vector<AssemblyPart> parts;  ///< List of parts
};

/**
 * @class CombinedTask
 * @brief Represents a combined task.
 */
class CombinedTask
{
public:
    /** @brief Default constructor. */
    CombinedTask() = default;

    /**
     * @brief Constructor to initialize a combined task.
     * @param input_station The assembly station.
     * @param input_parts List of parts involved in the task.
     */
    CombinedTask(u_int8_t input_station, std::vector<AssemblyPart> input_parts)
    {
        station = set_station(input_station);
        parts = input_parts;
    }

    /**
     * @brief Sets the assembly station.
     * @param input_station The station identifier.
     * @return The station as a string.
     */
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

    /** @brief Gets the station of the combined task. */
    std::string get_station() const { return station; }

    /** @brief Gets the list of parts involved in the task. */   
    const std::vector<AssemblyPart> &get_parts() const { return parts; }

private:
    std::string station; ///< Assembly station
    std::vector<AssemblyPart> parts; ///< List of parts
};

/**
 * @class Order
 * @brief Represents an order in the ARIAC system.
 */
class Order
{
public:
    /** @brief Default constructor. */
    // Order() = default;
    Order() : id("null") {}


    /**
     * @brief Constructor to initialize an order.
     * @param order_id Unique identifier for the order.
     * @param order_type Type of the order.
     */
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
/**
 * @brief Sets the type of the task.
 * 
 * This method sets the task type based on the provided input type. It maps the input integer 
 * to a corresponding string representing the task type (e.g., "KITTING", "ASSEMBLY", or "COMBINED").
 * If the input type is not recognized, it returns "no type added".
 * 
 * @param input_type The type identifier as an integer (defined in `ariac_msgs::msg::Order`).
 * @return A string representing the type of the task.
 */
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

    /** @brief Gets the order ID. */
    std::string get_id() const { return id; }

    /** @brief Gets the order type. */
    std::string get_type() const { return type; }

    /** @brief Gets the priority. */
    bool get_priority() const { return priority; }

    /** @brief Gets the kitting task object. */
    const KittingTask &get_kitting_task() const { return kitting_task; }

    /** @brief Gets the assembly task object. */
    const AssemblyTask &get_assembly_task() const { return assembly_task; }

    /** @brief Gets the combined task object. */
    const CombinedTask &get_combined_task() const { return combined_task; }

    bool is_valid() const {return id != "null";}

private:
    std::string id{}; ///< Unique identifier for the order
    std::string type{}; ///< Unique identifier for the order
    bool priority{};
    KittingTask kitting_task;
    AssemblyTask assembly_task;
    CombinedTask combined_task;
};

 /**
  * @class RetrieveOrders
  * @brief A ROS 2 node for retrieving and managing orders.
  */
class RetrieveOrders : public rclcpp::Node
{
public:
     /**
      * @brief Constructor for the RetrieveOrders node.
      * @param node_name Name of the node.
      */
    RetrieveOrders(std::string node_name) : Node(node_name)
    {
        order_subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10,
                                                                        std::bind(&RetrieveOrders::order_callback, this,
                                                                                  std::placeholders::_1));

        // RCLCPP_INFO(this->get_logger(),"subscriber created");
        
        tray_pose_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/detected_trays", 10,
            std::bind(&RetrieveOrders::tray_pose_callback, this,
                      std::placeholders::_1));

        // RCLCPP_INFO(this->get_logger()," tray subscriber created");

        // Create the timer callback function
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),  // every 5 seconds
            std::bind(&RetrieveOrders::order_processing_callback, this));

    }

     /**
      * @brief Callback function for processing incoming orders.
      * @param msg Shared pointer to the received order message.
      */
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    void tray_pose_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg );
    
    /**
      * @brief Displays order details.
      * @param order The order to be displayed.
      */
    void display_order(const Order &order);

    void order_processing_callback();

    const std::queue<Order>& get_normal_orders() const { return normal_orders; }

    const std::queue<Order>& get_priority_orders() const { return priority_orders; }

    // Pops and returns the next priority order
    Order pop_priority_order();

    // Pops and returns the next normal order
    Order pop_normal_order();

    void log_tray_poses(const Order& current_order);


private:
    // order_subscriber_
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subscriber_;

    // tray pose subscriber;;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr tray_pose_subscriber_;

    ///< Queue storing normal priority orders
    std::queue<Order> normal_orders;

    ///< Queue storing high priority orders.
    std::queue<Order> priority_orders;
    
    // timer callback
    rclcpp::TimerBase::SharedPtr timer_;

    ariac_msgs::msg::AdvancedLogicalCameraImage latest_tray_pose_;

    bool tray_pose_received_ = false;  // flag to check if any message has been received

};