# VLMQuery.msg
# Query message for visual language models

# Header for timestamp and frame information
Header header

# Image to process
sensor_msgs/Image image

# Text query or prompt
string text

# Task to perform (describe, vqa, detection, embed, embed_text, similarity)
string task

# Model to use (optional, defaults to the loaded model)
string model
