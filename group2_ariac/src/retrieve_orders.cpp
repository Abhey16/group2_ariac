#include "retrieve_orders.hpp"

// Callback function to process incoming order messages
void RetrieveOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{

    // Get Order Attributes

    // Create a vector to store kitting parts
    std::vector<KittingPart> kitting_part;

    // Loop through all parts in the kitting task and create KittingPart objects
    for (const auto &part_msg : msg->kitting_task.parts)
    {
        kitting_part.push_back(KittingPart(Part(part_msg.part.color, part_msg.part.type),
                                           part_msg.quadrant));
    }

    // Create KittingTask object
    KittingTask kitting(msg->kitting_task.agv_number,
                        msg->kitting_task.tray_id,
                        msg->kitting_task.destination,
                        kitting_part);

    // Create AssemblyPart Object
    std::vector<AssemblyPart> assembly_part;
    for (const auto &part_msg : msg->assembly_task.parts)
    {
        assembly_part.push_back(AssemblyPart(Part(part_msg.part.color, part_msg.part.type),
                                             part_msg.assembled_pose,
                                             part_msg.install_direction));
    }

    // Create AssemblyTask object
    AssemblyTask assembly(msg->assembly_task.agv_numbers,
                          msg->assembly_task.station,
                          assembly_part);

    // Create CombinedPart object
    std::vector<AssemblyPart> combined_part;
    for (const auto &part_msg : msg->combined_task.parts)
    {
        combined_part.push_back(AssemblyPart(Part(part_msg.part.color, part_msg.part.type),
                                             part_msg.assembled_pose,
                                             part_msg.install_direction));
    }
    // Create CombinedTask object
    CombinedTask combined(msg->combined_task.station,
                          combined_part);

    // Create Order object
    Order order(msg->id,
                msg->type, msg->priority,
                kitting,
                assembly,
                combined);

    // Storing orders in a queue
    if (msg->priority == true)
    {
        priority_orders.push(order); // If it's a priority order, push it to the priority queue
    }
    else
    {
        normal_orders.push(order); // If it's a normal order, push it to the normal queue

    }

    // Display the order details
    // RetrieveOrders::display_order(order);
}

void RetrieveOrders::tray_pose_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg )
{
    // auto tray_poses_vec = msg->tray_poses;

    // // for(const auto& poses : tray_poses_vec)
    // // {
    // //     RCLCPP_INFO(this->get_logger(), " Tray : %d", poses.id);
    // // }

    latest_tray_pose_ = *msg;
    tray_pose_received_ = true;
}


void RetrieveOrders::order_processing_callback()
{

    // Pop from priority queue
    Order current_order = pop_priority_order();

    // Check if it's valid
    if (current_order.is_valid()) {
        RCLCPP_INFO(this->get_logger(), "-Order ID: %s", current_order.get_id().c_str());

        // get the tray pose from subscriber
        if (tray_pose_received_ && bin_parts_received_) {
            RetrieveOrders::log_tray_poses(current_order);
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "No tray pose message received yet.");
        }

        // get bin parts

        if (bin_parts_received_)
        {
            RetrieveOrders::log_bin_parts(current_order);
        }
        else
        {   
            RCLCPP_WARN(this->get_logger(), "No bin_parts message received yet.");            
        }

        if (conveyor_parts_received_)
        {
            RetrieveOrders::log_conveyor_parts(current_order);
        }
        else
        {   
            RCLCPP_WARN(this->get_logger(), "No conveyor_parts message received yet.");            
        }

        return;
    }

    // Pop from normal queue if priority was empty
    current_order = pop_normal_order();
    if (current_order.is_valid()) {
        RCLCPP_INFO(this->get_logger(), "Order ID: %s", current_order.get_id().c_str());
        
        if (tray_pose_received_ && bin_parts_received_) {
            RetrieveOrders::log_tray_poses(current_order);
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "No tray pose message received yet.");
        }

        if (bin_parts_received_)
        {
            RetrieveOrders::log_bin_parts(current_order);
        }
        else
        {   
            RCLCPP_WARN(this->get_logger(), "No bin_parts message received yet.");            
        }

        if (conveyor_parts_received_)
        {
            RetrieveOrders::log_conveyor_parts(current_order);
        }
        else
        {   
            RCLCPP_WARN(this->get_logger(), "No conveyor_parts message received yet.");            
        }
        return;
    }

    // If both were empty
    RCLCPP_INFO(this->get_logger(), "No orders in either queue.");

}

void RetrieveOrders::bin_part_callback(const group2_msgs::msg::PartList::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Bin part callback");
    bin_parts_ = *msg;
    bin_parts_received_ = true;
}

void RetrieveOrders::conveyor_part_callback(const group2_msgs::msg::PartList::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Conveyor part callback");
    conveyor_parts_ = *msg;
    conveyor_parts_received_ = true;
}

Order RetrieveOrders::pop_priority_order() {
    if (!priority_orders.empty()) {
        Order next_order = priority_orders.front();
        priority_orders.pop();
        // RCLCPP_INFO(this->get_logger(), "Popped order: %s", next_order.get_id().c_str());
        return next_order;
    } 
    else 
    {
        // RCLCPP_INFO(this->get_logger(), " No priority orders to pop.");
        return Order();
    }
}

Order RetrieveOrders::pop_normal_order() {
    if (!normal_orders.empty()) {
        Order next_order = normal_orders.front();
        normal_orders.pop();
        // RCLCPP_INFO(this->get_logger(), "Popped order: %s", next_order.get_id().c_str());
        return next_order;
    } 
    else 
    {
        // RCLCPP_WARN(this->get_logger(), "No normal orders to pop.");
        return Order();
    }
}

void RetrieveOrders::display_order(const Order &order)
{
    // Display order on the terminal
    RCLCPP_INFO(this->get_logger(), "*****************************");
    RCLCPP_INFO(this->get_logger(), "- Kitting Task : %s", order.get_id().c_str());

    RCLCPP_INFO(this->get_logger(), "    - Priority : %d", order.get_priority());

    RCLCPP_INFO(this->get_logger(), "    - Tray : %d", order.get_kitting_task().get_tray_id());

    for (const auto &part : order.get_kitting_task().get_parts())
    {
        RCLCPP_INFO(this->get_logger(), "       - %s: %s %s",
                    part.get_quadrant().c_str(),
                    part.get_part().get_color().c_str(),
                    part.get_part().get_type().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "    - AGV : %d", order.get_kitting_task().get_agv_number());

    RCLCPP_INFO(this->get_logger(), "*****************************");
}

void RetrieveOrders::log_tray_poses(const Order& current_order)
{
    for (const auto& tray_pose : latest_tray_pose_.tray_poses) 
    {
        // RCLCPP_INFO(this->get_logger(), "Tray ID: %d", tray_pose.id);

        int tray_id = tray_pose.id;
        const auto& pose = tray_pose.pose;

        if (tray_id == current_order.get_kitting_task().get_tray_id())
        {

            RCLCPP_INFO(this->get_logger(), 
                "   -Tray ID %d: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]",
                tray_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            );
            
            RCLCPP_INFO(this->get_logger(), "   -Parts:");
        }

        else
        {
            continue;
        }

    }
} 

void RetrieveOrders::log_bin_parts(const Order& current_order)
{   
    for ( const auto& bin_part : bin_parts_.parts)
    {   
        for (const auto &part : current_order.get_kitting_task().get_parts())
        {   
            std::string type = bin_part.type;
            std::transform(type.begin(), type.end(), type.begin(), ::toupper);

            std::string color = bin_part.color;
            std::transform(color.begin(), color.end(), color.begin(), ::toupper);

            if (type == part.get_part().get_type().c_str() && color == part.get_part().get_color().c_str())
            {   
                
                // RCLCPP_INFO(this->get_logger(), "       - %s %s",
                // part.get_part().get_color().c_str(),
                // part.get_part().get_type().c_str());

                RCLCPP_INFO(this->get_logger(), "  - %s %s:", bin_part.color.c_str(), bin_part.type.c_str());
                RCLCPP_INFO(this->get_logger(), "    - Location: %s", bin_part.location.c_str());
                RCLCPP_INFO(this->get_logger(), "    - [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                            bin_part.position.x, bin_part.position.y, bin_part.position.z,
                            bin_part.orientation.x, bin_part.orientation.y, bin_part.orientation.z, bin_part.orientation.w);

                break;
            }

        }
        // if (bin_part.type == current_order.
    }
}

void RetrieveOrders::log_conveyor_parts(const Order& current_order)
{   
    for ( const auto& conveyor_part : conveyor_parts_.parts)
    {   
        for (const auto &part : current_order.get_kitting_task().get_parts())
        {   
            std::string type = conveyor_part.type;
            std::transform(type.begin(), type.end(), type.begin(), ::toupper);

            std::string color = conveyor_part.color;
            std::transform(color.begin(), color.end(), color.begin(), ::toupper);

            if (type == part.get_part().get_type().c_str() && color == part.get_part().get_color().c_str())
            {   
                // RCLCPP_INFO(this->get_logger(), "       - %s %s",
                // part.get_part().get_color().c_str(),
                // part.get_part().get_type().c_str());

                RCLCPP_INFO(this->get_logger(), "  - %s %s:", conveyor_part.color.c_str(), conveyor_part.type.c_str());
                RCLCPP_INFO(this->get_logger(), "    - Location: %s", conveyor_part.location.c_str());
                RCLCPP_INFO(this->get_logger(), "    - First detection: [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                            conveyor_part.initial_pos.x, conveyor_part.initial_pos.y, conveyor_part.initial_pos.z,
                            conveyor_part.orientation.x, conveyor_part.orientation.y, conveyor_part.orientation.z, conveyor_part.orientation.w);
                RCLCPP_INFO(this->get_logger(), "    - Current Position: [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                            conveyor_part.position.x, conveyor_part.position.y, conveyor_part.position.z,
                            conveyor_part.orientation.x, conveyor_part.orientation.y, conveyor_part.orientation.z, conveyor_part.orientation.w);                            
                RCLCPP_INFO(this->get_logger(), "    - Prediction [1s]: [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                            conveyor_part.predicted_pos_1.x, conveyor_part.predicted_pos_1.y, conveyor_part.predicted_pos_1.z,
                            conveyor_part.orientation.x, conveyor_part.orientation.y, conveyor_part.orientation.z, conveyor_part.orientation.w);  
                RCLCPP_INFO(this->get_logger(), "    - Prediction [2s]: [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                            conveyor_part.predicted_pos_2.x, conveyor_part.predicted_pos_2.y, conveyor_part.predicted_pos_2.z,
                            conveyor_part.orientation.x, conveyor_part.orientation.y, conveyor_part.orientation.z, conveyor_part.orientation.w);                            

                break;
            }
        }
        // if (conveyor_part.type == current_order.
    }
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create a RetrieveOrders node instance
    auto node = std::make_shared<RetrieveOrders>("retrieve_orders");

    // Use a multithreaded executor to allow multiple callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();
    
    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}
