from phi.agent import Agent
from phi.model.groq import Groq
from dotenv import load_dotenv
import gradio as gr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# Load environment variables
load_dotenv()

# ROS2 Node Class for Publishing Messages
class ROSPublisherNode(Node):
    def __init__(self):
        super().__init__('robot_navigation_agent_node')
        self.publisher = self.create_publisher(String, 'llm_waypoint', 10)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {message}')


# Initialize the agent
agent = Agent(
    model=Groq(id="llama-3.3-70b-versatile"),
    instructions=[
        "You are an AI agent providing navigation instructions to a mobile robot based on user needs.",
        "The kitchen is at (9.0, -3.5, 0.0), bedroom is at (-2.0, 2.0, 0), bathroom is at (-3.0, -3.0, 0), and the hall is at (2.0, -1.5, 0).",
        "When I ask you to bring something you should go to either kitchen, bedroom, or bathroom and bring the item to the hall.",
        "Let's say I ask you to bring water.",
        "The output instructions should be in the following format: ",
        "{'move1': (9.0, -3.5, 0.0), 'move2': (2.0, -1.5, 0.0),}",
        "Keep the response simple and make sure to provide all the move instructions in a single dictionary {}.",
    ],
    markdown=True,
    debug_mode=True,
)

# Initialize ROS2
rclpy.init()
node = ROSPublisherNode()

# Define a function for interaction
def agent_response(user_input):
    # Get agent response
    response = agent.print_response(user_input, stream=True)
    original_response = response
    # Extract the part of the response enclosed in curly braces
    response = response.split("{")[1].split("}")[0]
    formatted_response = f"{{{response}}}"
    # convert to dictionary
    formatted_response = dict(eval(formatted_response))

    formatted_response = json.dumps(formatted_response)
    print(f"Formatted Response: {formatted_response}")

    # Publish the agent's response to the ROS2 topic
    node.publish_message(formatted_response)

    # Return the formatted response for the Gradio interface
    return original_response

# Create a Gradio interface
interface = gr.Interface(
    fn=agent_response,  # Function to process user input
    inputs=gr.Textbox(label="User Command"),  # Input widget
    outputs=gr.Textbox(label="Robot Instructions"),  # Output widget
    title="Robot Navigation Agent",
    description="Provide commands to the robot agent, and it will generate navigation instructions. Agent's responses are published to a ROS2 topic.",
)

# Launch the interactive window
try:
    interface.launch()
finally:
    # Shutdown ROS2 gracefully
    rclpy.shutdown()
