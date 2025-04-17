from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="group2_ariac", 
                executable='start_competition_node',
                name='start_competition_node'
            ),
            # Node(
            #     package="group2_ariac", 
            #     executable='retrieve_orders',
            #     name='retrieve_orders'
            # ),
            # Node(
            #     package="group2_ariac", 
            #     executable='tray_detector',
            #     name='tray_detector'
            # ),            
            Node(
                package="group2_ariac", 
                executable='bin_parts_detector',
                name='bin_parts_detector',
                # prefix=["gdbserver localhost:3000"],
            ),   
            # Node(
            #     package="group2_ariac", 
            #     executable='ship_orders',
            #     name='ship_orders'
            # ),
            # Node(
            #     package="group2_ariac", 
            #     executable='complete_orders',
            #     name='complete_orders'
            # ),                                    
            # Node(
            #     package="group2_ariac", 
            #     executable='end_competition_node',
            #     name='end_competition_node'
            # ),
        ]
    )