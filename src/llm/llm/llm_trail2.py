from phi.agent import Agent
from phi.model.groq import Groq
from dotenv import load_dotenv
import gradio as gr

# Load environment variables
load_dotenv()

# Initialize the agent
agent = Agent(
    model=Groq(id="llama-3.3-70b-versatile"),
    instructions=[
        "You are an AI agent providing navigation instructions to a mobile robot based on user needs.",
        "The kitchen is at (9.0, -3.5, 0.0), bedroom is at (-2.0, 2.0, 0), bathroom is at (-3.0, -3.0, 0), and the hall is at (2.0, -1.5, 0).",
        "When I ask you to bring something you should go to either kitchen, bedroom, or bathroom and bring the item to the hall.",
        "Let's say I ask you to bring water.",
        "The output instructions should be in the following format: ",
        "{'move1': (9.0, -3.5, 0.0) # Moving to Kitchen, 'move2': (2.0, -1.5, 0.0) # Moving to Hall,}",
        "Keep the response simple and make sure to provide all the move instructions in a single dictionary {}.",
    ],
    markdown=True,
    debug_mode=True,
)

# Define a function for interaction
def agent_response(user_input):
    # print(agent.run(user_input))
    response = agent.print_response(user_input, stream=True)
    # get the part of response that is enclosed in curly braces
    response = response.split("{")[1].split("}")[0]
    return response
    # return agent.print_response(user_input, stream=False)

# Create a Gradio interface
interface = gr.Interface(
    fn=agent_response,  # Function to process user input
    inputs=gr.Textbox(label="User Command"),  # Input widget
    outputs=gr.Textbox(label="Robot Instructions"),  # Output widget
    title="Robot Navigation Agent",
    description="Provide commands to the robot agent, and it will generate navigation instructions.",
)

# Launch the interactive window
interface.launch()
