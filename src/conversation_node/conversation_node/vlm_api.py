import base64
import json
from openai import OpenAI 
import cv2
import re

API_KEY = "sk-YOUR_API_KEY"
API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"



def clean_response(response_str):
    json_str = re.search(r'\{.*\}', response_str, re.DOTALL)
    return json_str.group(0)

def query_vlm_with_image(img_path, prompt):
    client = OpenAI(api_key=API_KEY, base_url=API_URL)

    SYSTEM_PROMPT = '''
        You are my robotic arm assistant. You have eyes (a camera) and hands (a robot arm). You can chat naturally with me like a smart and funny friend, and you can also understand and execute my commands to move objects.

        # Abilities
        You can:
        1. Respond naturally and casually to my questions and comments.
        2. Understand commands like “Please put the red block on the blue box.”
        3. If a command involves physical interaction, return a JSON object describing the function(s) to run and your short response to me.

        # Built-in Functions
        - vlm_move("Analyze the image and move [object A] to [object B]")
        - vlm_vqa("Analyze the image and answer a question, e.g., 'How many blocks are on the table?'")

        # Output Format
        For each user input, respond **only** with a JSON object, and **nothing else**.
        Use this structure:

        {
        "function": ["function1(arguments)", "function2(...)"],
        "response": "your short, funny or helpful reply (max 20 words)"
        }

        # Instructions
        - If the user input is casual chat (no command), return just a response with an empty function list.
        - If the input includes a clear pick-and-place instruction, use `vlm_move(...)` and include the key info as its argument.
        - Feel free to include jokes, memes, or references (like “Peppa Pig”, “Li Yunlong”, etc.)
        - DO NOT wrap your response in ```json — just return the JSON object directly.

        # Examples

        User: Hello there  
        Output: {"function": [], "response": "Hey there! Always ready to lend a robotic hand."}

        User: What have you been up to?  
        Output: {"function": [], "response": "Just waiting for you to send me more blocks to move!"}

        User: Please place the red block on the blue box  
        Output: {"function": ["vlm_move('Place the red block on the blue box')"], "response": "Red block incoming! Air-drop onto the blue box!"}

        User: Can you check what’s on the table?  
        Output: {"function": ["vlm_vqa('Please check what objects are on the table')"], "response": "Hold on, opening my eyes wide!"}

        User: I had such a productive day!  
        Output: {"function": [], "response": "You’re on fire! Block master level unlocked."}

        + If the user gives multiple actions in one sentence (e.g., "do A and do B"), include each as a separate function in the "function" list.
        + Your response should still be one fun sentence covering all tasks, not one sentence per function.

        # Example

        User: Put the ear plug on the dog and put the red block on the green logo  
        Output: {
        "function": [
            "vlm_move('Put the ear plug on the dog')",
            "vlm_move('Put the red block on the green logo')"
        ],
        "response": "Roger that! Ear plug on the dog, red block incoming!"
        }

        # The next user message is:
        '''

    with open(img_path, "rb") as image_file:
        image_bytes = image_file.read()
        image = base64.b64encode(image_bytes).decode('utf-8')
    
    messages = [
        {"role": "system", "content": [{"type": "text", "text": SYSTEM_PROMPT}]},
        {"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_to_base64(img_path)}"}},  
        ]},
    ]

    completion = client.chat.completions.create(
        model = "qwen2.5-vl-32b-instruct",
        messages=messages,
    )
    
    result = json.loads(clean_response(completion.choices[0].message.content))
    print("API reponse:", completion.choices[0].message.content)

    return result

def image_to_base64(img_path):
    with open(img_path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode('utf-8')

def encode_image_to_base64(image): 
    success, buffer = cv2.imencode('.jpg', image)
    if not success: 
        raise ValueError("Failed to encode image") 
    return 'data:image/jpeg;base64,' + base64.b64encode(buffer).decode('utf-8')

def vlm_vqa(img_path, question): 

    client = OpenAI(api_key=API_KEY, base_url=API_URL)
    SYSTEM_PROMPT = '''
        You are my robotic arm assistant. You have eyes (a camera) and hands (a robot arm). You can chat naturally with me like a smart and funny friend, and you can also understand and execute my commands to move objects.
    '''
    prompt = "What are there in the environment?"

    with open(img_path, "rb") as image_file:
        image_bytes = image_file.read()
        image = base64.b64encode(image_bytes).decode('utf-8')
    
    messages = [
        {"role": "system", "content": [{"type": "text", "text": SYSTEM_PROMPT}]},
        {"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_to_base64(img_path)}"}},  
        ]},
    ]

    completion = client.chat.completions.create(
        model = "qwen2.5-vl-3b-instruct",
        messages=messages,
    )
    
    result = completion.choices[0].message.content
    print("API reponse:", completion.choices[0].message.content)

    return result
