# BOS_VLM: Visual Language Models for Brew Operating System

A lightweight, easy-to-use package for integrating visual language models with the Brew Operating System.

## Features

- Simple interface for multiple visual language models (CLIP, OWL-ViT, etc.)
- BOS topics for image inputs and language outputs
- Support for real-time inference on robot camera feeds
- Modular design for easy extension to new models
- Prebuilt Docker container for hassle-free setup
- Built-in support for common robotics tasks:
  - Visual question answering
  - Object grounding
  - Scene description
  - Visual reasoning

## Directory Structure

\\\
BOS_VLM/
+-- BOS_VLM/                      # Main package
¦   +-- __init__.py
¦   +-- models/                   # Model implementations
¦   ¦   +-- __init__.py
¦   ¦   +-- base_model.py         # Abstract base class
¦   ¦   +-- clip_model.py
¦   ¦   +-- owlvit_model.py
¦   ¦   +-- ...
¦   +-- bos/                      # BOS integration
¦   ¦   +-- __init__.py
¦   ¦   +-- node.py               # Main BOS node
¦   ¦   +-- topics.py             # Topic definitions
¦   ¦   +-- utils.py
¦   +-- utils/
¦       +-- __init__.py
¦       +-- image_processing.py
¦       +-- text_processing.py
+-- config/                       # Configuration files
¦   +-- default.yaml              # Default configuration
¦   +-- models/                   # Model-specific configurations
¦       +-- clip.yaml
¦       +-- owlvit.yaml
+-- launch/                       # BOS launch files
¦   +-- vlm_node.launch
¦   +-- demo.launch
+-- scripts/                      # Utility scripts
¦   +-- download_models.sh
¦   +-- benchmark.py
+-- examples/                     # Usage examples
¦   +-- simple_query.py
¦   +-- object_detection.py
¦   +-- interactive_demo.py
+-- tests/                        # Unit and integration tests
¦   +-- test_models.py
¦   +-- test_bos_integration.py
+-- docs/                         # Documentation
¦   +-- installation.md
¦   +-- models.md
¦   +-- bos_integration.md
¦   +-- examples.md
+-- Dockerfile                    # Docker setup
+-- setup.py                      # Package setup
+-- requirements.txt              # Python dependencies
+-- LICENSE                       # Open source license
+-- README.md                     # Main documentation
\\\

## Installation

\\\ash
# Clone the repository
git clone https://github.com/homebrewroboticsclub/BOS_VLM.git
cd BOS_VLM

# Install dependencies
pip install -e .

# Download pre-trained models
./scripts/download_models.sh

# Build BOS package
cd ~/brew_ws/
make
\\\

## Quick Start

### Launch the BOS node

\\\ash
brew launch BOS_VLM vlm_node.launch model:=clip
\\\

### Query the model

\\\python
import brew
from BOS_VLM.msg import VLMQuery, VLMResponse
from sensor_msgs.msg import Image

# Initialize BOS node
brew.init_node('vlm_client')

# Set up publishers and subscribers
query_pub = brew.Publisher('/vlm/query', VLMQuery, queue_size=10)
response_sub = brew.Subscriber('/vlm/response', VLMResponse, callback)

# Create a query
query = VLMQuery()
query.image = current_image  # sensor_msgs/Image
query.text = "What objects can I pick up in this scene?"
query.model = "clip"

# Send the query
query_pub.publish(query)
\\\

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
