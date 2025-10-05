from langchain_anthropic import ChatAnthropic
from langchain.agents import tool
from rosa import ROSA
from rosa.prompts import RobotSystemPrompts
import os
import pathlib
import time
import subprocess
from typing import Tuple
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

node = None
vel_publisher = None
explore_publisher = None
navigate_to_pose_action_client = None

def execute_ros_command(command: str) -> Tuple[bool, str]:
    cmd = command.split(" ")
    if len(cmd) < 2 or cmd[0] != "ros2":
        raise ValueError("Invalid ROS2 command: " + command)
    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)

def _get_maps_dir() -> str:
    maps_dir = "/home/ros/rap/Gruppe2/maps"
    pathlib.Path(maps_dir).mkdir(parents=True, exist_ok=True)
    return maps_dir

LOCATIONS = {
    "gym": {"position": {"x": 1.9517535073729964, "y": 4.359393291484201, "z": 0.0},
            "orientation": {"x": 1.6302566310137402e-08, "y": 2.9703213238180324e-08, "z": -0.07945176214102775, "w": 0.9968387118750377}},
    "kitchen": {"position": {"x": 7.353217566768062, "y": -3.458078519447155, "z": 0.0},
                "orientation": {"x": 1.611930208234276e-08, "y": 2.980589390984495e-08, "z": -0.07325043342926793, "w": 0.997313578571165}},
    "living room": {"position": {"x": 1.084137940689, "y": -0.383112079564818, "z": 0.0},
                    "orientation": {"x": 3.316520260505064e-08, "y": 6.931688679143018e-09, "z": -0.8089064668855616, "w": 0.5879373502599718}},
    "office": {"position": {"x": -4.9521764504716765, "y": -3.573205806403106, "z": 0.0},
               "orientation": {"x": -3.238093524948138e-08, "y": 9.961584542476143e-09, "z": 0.9923116132365216, "w": -0.1237645435329962}},
    "bedroom": {"position": {"x": -4.002267652240865, "y": -0.060121871401907084, "z": 0.0},
                "orientation": {"x": -2.1636165143756515e-08, "y": 2.6069771799291994e-08, "z": 0.8980250477792399, "w": 0.43994433007039957}},
}

@tool
def send_vel(velocity: float) -> str:
    global vel_publisher
    twist = Twist()
    twist.linear.x = velocity
    vel_publisher.publish(twist)
    return "Velocity set to %s" % velocity

@tool
def stop() -> str:
    global vel_publisher
    twist = Twist()
    vel_publisher.publish(twist)
    return "Robot stopped"

@tool
def toggle_auto_exploration(resume_exploration: bool) -> str:
    global explore_publisher
    msg = Bool()
    msg.data = resume_exploration
    explore_publisher.publish(msg)
    return "Autonomous exploration started/resumed." if resume_exploration else "Autonomous exploration stopped/paused."

@tool
def navigate_to_pose(x: float, y: float, z_orientation: float, w_orientation: float) -> str:
    global navigate_to_pose_action_client, node
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation
    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}."

@tool
def navigate_relative(x: float, y: float, z_orientation: float, w_orientation: float) -> str:
    global navigate_to_pose_action_client, node
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "base_link"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation
    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Relative navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}."

@tool
def save_map(map_name: str) -> str:
    maps_dir = _get_maps_dir()
    if not os.path.isdir(maps_dir):
        return f"Error: Maps directory {maps_dir} could not be accessed or created."
    filepath_prefix = os.path.join(maps_dir, map_name)
    cmd = f"ros2 run nav2_map_server map_saver_cli -f '{filepath_prefix}' --ros-args -r map:=/summit/map"
    success, output = execute_ros_command(cmd)
    if success:
        return f"Map successfully saved as {map_name} in {maps_dir}" if "Map saved to" in output else f"Map saving process initiated for {map_name} in {maps_dir}. Output: {output}"
    else:
        return f"Failed to save map {map_name} in {maps_dir}. Error: {output}"

@tool
def list_saved_maps() -> str:
    maps_dir = _get_maps_dir()
    if not os.path.isdir(maps_dir):
        return "Maps directory not found or is not a directory."
    try:
        files = os.listdir(maps_dir)
        map_files = [f[:-5] for f in files if f.endswith(".yaml") and os.path.isfile(os.path.join(maps_dir, f))]
        return f"Available maps: {', '.join(map_files)}" if map_files else "No saved maps found in the maps directory."
    except Exception as e:
        return f"Error listing maps: {e}"

@tool
def get_location_names() -> str:
    return f"Available locations: {', '.join(LOCATIONS.keys())}"

@tool
def navigate_to_location_by_name(location_name: str) -> str:
    global navigate_to_pose_action_client, node
    location_name_lower = location_name.lower()
    if location_name_lower not in LOCATIONS:
        return f"Location '{location_name}' not found. Available locations are: {', '.join(LOCATIONS.keys())}"
    loc_data = LOCATIONS[location_name_lower]
    pos = loc_data["position"]
    orient = loc_data["orientation"]
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = pos["x"]
    goal_msg.pose.pose.position.y = pos["y"]
    goal_msg.pose.pose.orientation.z = orient["z"]
    goal_msg.pose.pose.orientation.w = orient["w"]
    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to location '{location_name}'. Position: {pos}, Orientation: {orient}."

def main():
    global node, vel_publisher, explore_publisher, navigate_to_pose_action_client
    print("Hi from rosa_summit.")

    rclpy.init()
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("rosa_summit_node", parameter_overrides=[sim_time_param])
    vel_publisher = node.create_publisher(Twist, "/summit/cmd_vel", 10)
    explore_publisher = node.create_publisher(Bool, "/summit/explore/resume", 10)
    navigate_to_pose_action_client = ActionClient(node, NavigateToPose, "/summit/navigate_to_pose")


    try:
        with open("/home/ros/rap/Gruppe2/api-key.txt", "r") as f:
            api_key = f.read().strip().split("\n")[-1]
        llm = ChatAnthropic(
            model="claude-3-5-sonnet-20240620",
            temperature=0,
            anthropic_api_key=api_key,
            max_tokens=4096,
        )
    except Exception as e:
        print(f"Error reading Claude API key or initializing client: {e}")
        return

    prompt = RobotSystemPrompts()
    prompt.embodiment = (
        "You are a helpful robot named Summit, designed to assist users in a simulated environment. You can navigate, explore, and interact with the environment using various tools."
    )

    ensure_shutdown(ROSA)

    agent = ROSA(
        ros_version=2,
        llm=llm,
        tools=[
            send_vel,
            stop,
            toggle_auto_exploration,
            navigate_to_pose,
            navigate_relative,
            save_map,
            list_saved_maps,
            get_location_names,
            navigate_to_location_by_name,
        ],
        prompts=prompt,
    )

    print("Type 'exit' or 'quit' to end the program")

    try:
        while True:
            msg = input("Enter your request: ")
            if msg.lower() in ["exit", "quit"]:
                break
            print("Request sent")
            try:
                
                res = agent.invoke(msg)
                if isinstance(res, list) and res:
                    result = res[0]
                else:
                    result = res
                if isinstance(result, dict) and "text" in result:
                    print(f"Reply: {result['text']}")
                else:
                    print(f"Reply: {result}")
            except Exception as e:
                print(f"Error generating LLM response: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

    agent.shutdown()
    print("Bye from rosa_summit.")

if __name__ == "__main__":
    main()
