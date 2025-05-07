import base64
import json
from openai import OpenAI 
import re

API_KEY = "sk-YOUR_API_KEY"
API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"



def clean_response(response_str):
    json_str = re.search(r'\{.*\}', response_str, re.DOTALL)
    return json_str.group(0)

def find_coords(img_path, prompt):
    client = OpenAI(api_key=API_KEY, base_url=API_URL)

    SYSTEM_PROMPT = '''
    You will receive an instruction intended for a robotic arm. Your task is to extract the starting object and the destination object from the given instruction, identify their respective top-left and bottom-right pixel coordinates within the provided image, and return the data in JSON format. 

    For example, if the instruction is: "Please place the red block on the house sketch."
    You should return the following JSON structure: 
    {
    "start": "red block",
    "start_coordinates": [[xmin, ymin, xmax, ymax]],
    "end": "green block",
    "end_coordinates": [[xmin, ymin, xmax, ymax]],
     }

    Ensure that your response contains only the JSON output, without any additional text. 

    Instruction:
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
        model = "qwen2.5-vl-3b-instruct",
        messages=messages,
    )
    
    result = json.loads(clean_response(completion.choices[0].message.content))
    print("API reponse:", completion.choices[0].message.content)

    return result

def image_to_base64(img_path):
    with open(img_path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode('utf-8')
        