# VLMResponse.msg
# Response message from visual language models

# Header for timestamp and frame information
Header header

# Text response
string text

# Confidence score (0-1)
float32 confidence

# Original query that produced this response
VLMQuery query
