# ESE-498 From Language to Action: A Vision-Language and Kinematics-Guided Framework for 6-DOF Robotic Arm Manipulation

[Project Website](https://sites.wustl.edu/fromlanguagetoaction/)

## Overview

This repository contains an open-source implementation of a modular ROS 2-based robotic manipulation system that integrates Vision-Language Models (VLMs), image segmentation, PCA-based 2D grasp frame estimation, and voice-driven interaction.

As the Electrical and Systems Engineering undergraduate capstone at Washington University in St.Louis (WashU), this project developed an intelligent manipulation framework for a 6-DOF robotic arm, bridging classical robotics with modern vision-language reasoning. The system enables a myCobot 280 Pi robot to interpret natural language instructions, perceive its environment through camera input, and execute accurate, kinematically guided pick-and-place tasks.

Key features include:
- **Natural language understanding** using Qwen2.5-VL for instruction parsing and object detection.
- **2D hand-eye calibration** to align image-space detections with robot-frame coordinates.
- **Adaptive grasping** via EfficientSAM segmentation and PCA-based local frame prediction for non-uniform objects.
- **Voice interface** powered by FastAPI and local TTS/STT modules, enabling hands-free control.
- **Feedback-based correction** for accurate placement, achieving 0.11 cm uncertainty.

Designed for modularity, affordability, and educational use, the system is well-suited for classroom demos (e.g., ESE-3050, ESE-446) and provides a strong foundation for future extensions into 3D manipulation with RGB-D sensing and 6-DOF grasp planning.

---

## ROS 2 Framework Overview

###  Set up API Key 
This implementation uses Qwen2.5-VL from Alibaba. Users may request an API key from [Aliyun](https://www.alibabacloud.com/help/en/model-studio/use-qwen-by-calling-api). 

Update API_KEY at `src/conversation_node/conversation_node/vlm_api.py` and `src/vlm_detection_node/vlm_detection_node/vlm_api.py`. 

###  Launch the Full ROS 2 System (with VLM + Voice Interaction)

Make sure \src is at your ROS 2 workspace :)

Open **four ros 2 terminals** and run the following in each:


```bash
cd ~/ros2_ws
ros2 run camera_node camera_node
ros2 run vlm_detection_node vlm_detection_node
ros2 run execution_node execution_node
ros2 run conversation_node conversation_node
```

In src/conversation_node/conversation_node/conversation_node.py, set USE_VOICE_INPUT = True when using voice conversation. Set USE_VOICE_INPUT = False when typing instructions through keyboard. 

In main_node terminal, 
type the instruction, i.e "put the ear plug on the dog". 


#### Running the API and Voice Servers (Laptop Side)

On the laptop server (in the same network as the robot), start the segmentation and voice servers in separate terminals:

##### 1. EfficientSAM FastAPI Server

- Clone the [EfficientSAM repository](https://github.com/yformer/EfficientSAM)
- Copy `fastapi_grasp_server.py` into the **root directory** of the EfficientSAM repo

Then launch the segmentation API with:

```bash
uvicorn fastapi_grasp_server:app --host 0.0.0.0 --port 8001
```

Don't forget to update the laptop's ip address at `src/conversation_node/conversation_node/conversation_node.py` and `src/vlm_detection_node/vlm_detection_node/frame_label_api.py`. 
##### 2. Voice FastAPI Server

The following server relies on built-in or external microphone and speaker of the laptop/PC. 

```bash
python voice_server.py # default --port 8000
```

### For the simplified prototype: 

```bash
cd ~/ros2_ws
ros2 run camera_node camera_node
ros2 run vlm_detection_node vlm_detection_node
ros2 run execution_node execution_node
ros2 run main_node main_node
```

## Evaluating 2D Frame Prediction on the Cornell Grasp Dataset

To evaluate our segmentation + PCA-based grasp frame prediction method:

1. Clone the [EfficientSAM repository](https://github.com/yformer/EfficientSAM).
2. Download the [Cornell Grasp Dataset from Kaggle](https://www.kaggle.com/datasets/oneoneliu/cornell-grasp?resource=download).
3. Copy `frame_label_test.ipynb` into the `EfficientSAM/notebooks/` folder.
4. Run the notebook to evaluate the 2D grasp label predictions using EfficientSAM + PCA.
