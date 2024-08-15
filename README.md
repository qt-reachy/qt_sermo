# QT_Whispercpp

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
- Local Large Language Models are being run through [Ollama](https://github.com/ollama/ollama) in [Jetson container](https://www.jetson-ai-lab.com/tutorial_ollama.html)/*Tailored Docker*
    - Currently using Llama 3 7B
- Make sure IP to Jetson Nano Orin is correct and both the robot and the Jetson Orin Nano are on the same local network.

1. 

# WIP
- ~~Integrate *brain* using LLM and Jetson Orin Nano~~
- Clean TTS, currently LLM reply is too verbose for ROS Acapela functionality.
- Ollama tools
    - Await Jetson container update for Llama 3.1 and tools functionality
    - Compare with Vector creation and RAG
- Use through tablets and LuxAI Studio
- Analytics


## issues
### Major
- Whispercpp is run through bash
    - Is there a Python library that I am missing?
### Minor
- Using absolute paths on QT
- Ollama is *slow*
    - Explore *faster* Jetson containers [Small LLM](https://www.jetson-ai-lab.com/tutorial_slm.html) & [NanoLLM](https://www.jetson-ai-lab.com/tutorial_nano-llm.html)
    - Use lighter models, like Phi3 is another alternative
- LLM reply is wrong format
    - Explored grammar, not available in Ollama...
    - Regex hacks?
- Tracking could be better integrated
