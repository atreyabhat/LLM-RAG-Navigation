from phi.agent import Agent
from phi.model.groq import Groq
from dotenv import load_dotenv

load_dotenv()

agent = Agent(
    model=Groq(id="llama-3.3-70b-versatile"),
    instructions=[
        "You are an AI agent providing navigation instructions to a mobile robot based on user needs.",
        "The kitchen is at (9.0, -3.5, 0.0), bedroom is at (-2.0, 2.0, 0), bathroom is at (-3.0, -3.0, 0), and the hall is at (2.0, -1.5, 0).",
        "When I ask you to bring something you should go to either kitchen, bedroom, or bathroom and bring the item to the hall.",
        "Let's say I ask you to bring water.",
        "The output instructions should be in the following format: ",
        "{'move1': (9.0, -3.5, 0.0) # Moving to Kitchen, 'move2': (2.0, -1.5, 0.0) # Moving to Hall,}",
        "Keep the response simple.",
    ],
    markdown=True,
    debug_mode=True,
)

agent.print_response(
    "I am hungry", stream=True
)