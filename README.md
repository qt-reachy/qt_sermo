# Q_Whispercpp

[Whispercpp](https://github.com/ggerganov/whisper.cpp) for [QT robot](https://docs.luxai.com/docs/intro_code), integrated with ROS and external rPI 4 with [ReSpeaker](https://wiki.seeedstudio.com/ReSpeaker_4_Mic_Array_for_Raspberry_Pi/)

# Setup

## Whispercpp
- Running on QT NUC Intel® NUC  i7 4-core @ 4.4GHz, 32 GB RAM, 512 GB SSD

1. Create ROS project [LuxAI Documentation](https://docs.luxai.com/docs/tutorials/python/python_ros_project)
2. Clone Whispercpp into project
3. Build (and test Whispercpp)
4. Use files in this repo

## Nvidia Jetson Orin & LLM
- Local Large Language Models is being run through Ollama Nvidia Container on a [Nvidia Jetson Orin Nano Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/), 8GB with 1tb SSD.
- Local Large Language Models are being run through Ollama in jetson container/*Tailored Docker*
    - Currently using Llama 3 7B
- Make sure IP to Jetson Nano Orin is correct and both the robot and the Jetson Orin Nano are on the same local network.

# WIP
- Integrate *brain* using LLM and Jetson Orin Nano
- Use through tablets and LuxAI Studio

## issues
### Major
- Ollama is *slow*
### Minor
- Using absolute paths on QT

