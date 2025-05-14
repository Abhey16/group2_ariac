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
    // if the current process needs to stopped
    bool stop_flag = false;

    RCLCPP_INFO(this->get_logger(), "******Status of Ongoing Order: %d *****", ongoing_order_);

    // check if there is an ongoing order
    if (ongoing_order_)
    {   
        // check if the ongoing_order is not priority
        if(!current_priority)
        {
            // check if the next order is priority, stop ongoing order and update it
            if (!priority_orders.empty())
            {
                stop_flag = true;
                RCLCPP_INFO(this->get_logger(), "*********Status: Current Order is stopped*******");
            }

            // if ongoing order is not a priority & next order is not a priority, dont process order
            else
            {
                return;
            }
    
        }

        // if ongoing order is a priority, dont process next order
        else
        {
            return;
        }
    }

    if (!priority_orders.empty())
    {
        // Pop from priority queue
        Order current_order = get_priority_order();

        // Check if it's valid
        if (current_order.is_valid()) {
            RCLCPP_INFO(this->get_logger(), "-Order ID: %s", current_order.get_id().c_str());

            // Ongoing order
            ongoing_order_ = true;

            // get the tray pose from subscriber
            // if (tray_pose_received_ && bin_parts_received_)
            if (!latest_tray_pose_.tray_poses.empty()) {
                RetrieveOrders::get_tray_poses(current_order);
            } 
            else 
            {
                RCLCPP_WARN(this->get_logger(), "No tray pose message received yet.");
            }

            // get bin parts

            if (!bin_parts_.parts.empty())
            {
                RetrieveOrders::get_bin_parts(current_order);
            }
            else
            {   
                RCLCPP_WARN(this->get_logger(), "No bin_parts message received yet.");            
            }

            if (!conveyor_parts_.parts.empty())
            {
                RetrieveOrders::get_conveyor_parts(current_order);
            }
            else
            {   
                RCLCPP_WARN(this->get_logger(), "No conveyor_parts message received yet.");            
            }

            // simulating the order is completed

            // processing the task
            task_processing();

            pop_priority_order();
            ongoing_order_ = false;
            
            return;
        }
    }
    
    else
    {
        // Pop from normal queue if priority was empty
        Order current_order = get_normal_order();
        if (current_order.is_valid()) {
            RCLCPP_INFO(this->get_logger(), "Order ID: %s", current_order.get_id().c_str());

            // Ongoing order
            ongoing_order_ = true;
            
            // RCLCPP_INFO(this->get_logger(), "tray_pose_received_ : %d", !latest_tray_pose_.tray_poses.empty());
            if (!latest_tray_pose_.tray_poses.empty()) {
                RetrieveOrders::get_tray_poses(current_order);
            } 
            else 
            {
                RCLCPP_WARN(this->get_logger(), "No tray pose message received yet.");
            }

            // RCLCPP_INFO(this->get_logger(), "tray_pose_received_ : %d",!bin_parts_.parts.empty());
            if (!bin_parts_.parts.empty())
            {
                RetrieveOrders::get_bin_parts(current_order);
            }
            else
            {   
                RCLCPP_WARN(this->get_logger(), "No bin_parts message received yet.");            
            }

            // RCLCPP_INFO(this->get_logger(), "conveyor_parts_received_ : %d", !conveyor_parts_.parts.empty());
            if (!conveyor_parts_.parts.empty())
            {
                RetrieveOrders::get_conveyor_parts(current_order);
            }
            else
            {   
                RCLCPP_WARN(this->get_logger(), "No conveyor_parts message received yet.");            
            }

            // Simulating the order is completed

            // processing the task
            task_processing();

            pop_normal_order();

            ongoing_order_ = false;

            return;
        }
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

Order RetrieveOrders::get_priority_order() {
    if (!priority_orders.empty()) {
        Order next_order = priority_orders.front();
        return next_order;
    } 
    else 
    {
        // RCLCPP_INFO(this->get_logger(), " No priority orders to pop.");
        return Order();
    }
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

Order RetrieveOrders::get_normal_order() {
    if (!normal_orders.empty()) {
        Order next_order = normal_orders.front();
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

void RetrieveOrders::get_tray_poses(const Order& current_order)
{
    for (const auto& tray_pose : latest_tray_pose_.tray_poses) 
    {
        // RCLCPP_INFO(this->get_logger(), "Tray ID: %d", tray_pose.id);

        int tray_id = tray_pose.id;
        const auto& pose = tray_pose.pose;
        int32_t agv_number = current_order.get_kitting_task().get_agv_number();

        if (tray_id == current_order.get_kitting_task().get_tray_id())
        {

            // Display the tray poses recieved
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

            // Creating Task object
            Task object("tray",pose,agv_number);

            // pushing it into a queue
            task_queue.push(object);

            RCLCPP_INFO(this->get_logger(), "   -Parts:");
        }

        else
        {
            continue;
        }

    }
} 

void RetrieveOrders::get_bin_parts(const Order& current_order)
{   
    for ( const auto& bin_part : bin_parts_.parts)
    {   
        for (const auto &part : current_order.get_kitting_task().get_parts())
        {   
            std::string type = bin_part.type;
            std::transform(type.begin(), type.end(), type.begin(), ::toupper);

            std::string color = bin_part.color;
            std::transform(color.begin(), color.end(), color.begin(), ::toupper);

            std::string quadrant_number = part.get_quadrant();

            if (type == part.get_part().get_type().c_str() && color == part.get_part().get_color().c_str())
            {   
                // Display the detected bin parts

                RCLCPP_INFO(this->get_logger(), "  - %s %s:", bin_part.color.c_str(), bin_part.type.c_str());
                RCLCPP_INFO(this->get_logger(), "    - Location: %s", bin_part.location.c_str());
                RCLCPP_INFO(this->get_logger(), "    - [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                            bin_part.position.x, bin_part.position.y, bin_part.position.z,
                            bin_part.orientation.x, bin_part.orientation.y, bin_part.orientation.z, bin_part.orientation.w);

                // Creating Task object
                Task object("bin",parts_pose_to_geometry(bin_part),quadrant_number);

                // pushing it into a queue
                task_queue.push(object);

                break;
            }

        }
        // if (bin_part.type == current_order.
    }
}

void RetrieveOrders::get_conveyor_parts(const Order& current_order)
{   
    for ( const auto& conveyor_part : conveyor_parts_.parts)
    {   
        for (const auto &part : current_order.get_kitting_task().get_parts())
        {   
            std::string type = conveyor_part.type;
            std::transform(type.begin(), type.end(), type.begin(), ::toupper);

            std::string color = conveyor_part.color;
            std::transform(color.begin(), color.end(), color.begin(), ::toupper);

            std::string quadrant_number = part.get_quadrant();

            if (type == part.get_part().get_type().c_str() && color == part.get_part().get_color().c_str())
            {   
                // Display conveyor belt parts

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


                // Creating Task object
                Task object("conveyor",parts_pose_to_geometry(conveyor_part),quadrant_number);

                // pushing it into a queue
                task_queue.push(object);

                break;
            }
        }
        // if (conveyor_part.type == current_order.
    }
}

void RetrieveOrders::task_processing()
{
    while (!task_queue.empty())
    {
        Task current_task = task_queue.front();

        // call function to perform set task for every object
        RCLCPP_INFO(this->get_logger(), 
            "   -Pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]",
            current_task.get_pose().position.x,
            current_task.get_pose().position.y,
            current_task.get_pose().position.z,
            current_task.get_pose().orientation.x,
            current_task.get_pose().orientation.y,
            current_task.get_pose().orientation.z,
            current_task.get_pose().orientation.w
            );
        
        if (current_task.get_type() == "tray")
            move_it_pose(current_task.get_type(),current_task.get_pose(),current_task.get_agv());
        
        else
            move_it_pose(current_task.get_type(),current_task.get_pose(),current_task.get_quadrant());
        
        // move_it_pose();
            
        // call move method

        // call pick method

        // call place method
            
        RCLCPP_INFO(this->get_logger(), "task poped");

        task_queue.pop();
    }
}

// Helper function to convert parts pose to geometry_msgs::msg::Pose
geometry_msgs::msg::Pose RetrieveOrders::parts_pose_to_geometry(const group2_msgs::msg::Part &part_pose)
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = part_pose.position.x;
    pose.position.y = part_pose.position.y;
    pose.position.z = part_pose.position.z;
    pose.orientation.x = part_pose.orientation.x;
    pose.orientation.y = part_pose.orientation.y;
    pose.orientation.z = part_pose.orientation.z;
    pose.orientation.w = part_pose.orientation.w;

    return pose;
}

void RetrieveOrders::move_it_pose(std::string type,geometry_msgs::msg::Pose pose,int32_t agv)
{
    while (!move_it_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
    }

    auto request = std::make_shared<group2_msgs::srv::Pose::Request>();
    request->type = type;
    request->pose = pose;
    request->agv_number = agv;
    
    auto result = move_it_client_->async_send_request(
            request, std::bind(&RetrieveOrders::callback_move_it_pose, this, std::placeholders::_1));


}

void RetrieveOrders::move_it_pose(std::string type,geometry_msgs::msg::Pose pose, std::string quadrant)
{
    while (!move_it_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
    }

    auto request = std::make_shared<group2_msgs::srv::Pose::Request>();
    request->type = type;
    request->pose = pose;
    request->quadrant = quadrant;

    auto result = move_it_client_->async_send_request(
            request, std::bind(&RetrieveOrders::callback_move_it_pose, this, std::placeholders::_1));


}


void RetrieveOrders::callback_move_it_pose(rclcpp::Client<group2_msgs::srv::Pose>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response : %d", response->status);
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
