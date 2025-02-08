import pandas as pd
from langchain.vectorstores.faiss import FAISS
from sentence_transformers import SentenceTransformer
import faiss
import numpy as np
from groq import Groq
from dotenv import load_dotenv
import gradio as gr
import gradio as gr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

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

# Initialize ROS2
rclpy.init()
node = ROSPublisherNode()

# Initialize Groq client
client = Groq()

# Load Excel data
df = pd.read_excel("objects_locations.xlsx")

# Use a free embedding model
embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
embeddings = [embedding_model.encode(text) for text in df["Object"].tolist()]

# Convert embeddings to a NumPy array
embedding_array = np.array(embeddings, dtype="float32")

# Create a FAISS index
index = faiss.IndexFlatL2(embedding_array.shape[1])
index.add(embedding_array)

# Prepare the documents and metadata
docs = df["Object"].tolist()
docstore = {}
index_to_docstore_id = {}

for i, (obj, loc, coord) in enumerate(zip(df["Object"], df["Location"], df["Coordinates"])):
    doc_id = str(i)
    docstore[doc_id] = {
        "object": obj,
        "location": loc,
        "coordinates": coord,
        "content": obj 
    }
    index_to_docstore_id[i] = doc_id

# Create the FAISS vectorstore 
vectorstore = FAISS(
    embedding_function=embedding_model.encode,
    index=index,
    docstore=docstore,
    index_to_docstore_id=index_to_docstore_id
)

# Function to retrieve context
def retrieve_context(query):
    query_embedding = embedding_model.encode(query).reshape(1, -1).astype("float32")
    distances, indices = index.search(query_embedding, k=1)
    if indices[0][0] != -1:
        doc_id = index_to_docstore_id[indices[0][0]]
        doc = docstore[doc_id]
        return {
            "object": doc['object'],
            "location": doc['location'],
            "coordinates": tuple(map(float, doc['coordinates'].strip('()').split(',')))
        }
    return None

# Dynamic system prompt
def generate_system_prompt():
    context_data = [
        retrieve_context(obj) for obj in df["Object"].tolist()
        if retrieve_context(obj) is not None
    ]
    locations = {entry["object"]: entry["coordinates"] for entry in context_data}
    return (
        "You are an AI agent providing navigation instructions to a mobile robot based on user needs.\n"
        f"The following are object locations: {locations}.\n"
        "When I ask you to bring something, generate navigation instructions to fetch it. If destinaton is not specified, get it to hall at (2.0, -1.5, 0.0)"
        "Let's say I ask you to bring water."
        "The output instructions should be in the following format: "
        "{'move1': (9.0, -3.5, 0.0), 'move2': (2.0, -1.5, 0.0),}"
        "Keep the response simple, very simple, dont explain too much and make sure to provide all the move instructions in a single dictionary {}."
    )

# Function to query the model
def query_llama(prompt):
    system_prompt = generate_system_prompt()
    completion = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt},
        ],
        temperature=1,
        max_tokens=1024,
        top_p=1,
        stream=True,
    )

    response = ""
    for chunk in completion:
        response += chunk.choices[0].delta.content or ""
    return response

# RAG pipeline
def get_navigation_response(user_query):
    context = retrieve_context(user_query)
    # print(context)
    if context:
        prompt = (
            f"Based on the following database context: Object: {context['object']}, Location: {context['location']}, Coordinates: {context['coordinates']}.\n"
            f"User request: {user_query}.\n"
            "Generate navigation instructions."
        )
        llm_output = query_llama(prompt)
        response = llm_output.split("{")[1].split("}")[0]
        formatted_response = f"{{{response}}}"
        # convert to dictionary
        formatted_response = dict(eval(formatted_response))

        formatted_response = json.dumps(formatted_response)
        print(f"Formatted Response: {formatted_response}")
        node.publish_message(formatted_response)
        return str(context) + llm_output
    return "No relevant objects found."

interface = gr.Interface(
    fn=get_navigation_response,  # Function to process user input
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

# response = get_navigation_response("I want to enjoy my wine while I take a bath.")
# print("Response:", response)
# print("-------------------------------------------------------------------")

# response = get_navigation_response("I am bored without instagram")
# print("Response:", response)
# print("-------------------------------------------------------------------")


# response = get_navigation_response("Set up a workspace in the study with my laptop, a glass of water, and a book")
# print("Response:", response)
# print("-------------------------------------------------------------------")

# response = get_navigation_response("I am in the mood for biriyani")
# print("Response:", response)
# print("-------------------------------------------------------------------")
