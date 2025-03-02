#!/usr/bin/env python3
"""
Main BOS node for the Visual Language Model interface.
"""
import os
import sys
import yaml
import brew
import numpy as np
from cv_bridge import CvBridge
from typing import Dict, Any

# BOS message types
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from BOS_VLM.msg import VLMQuery, VLMResponse, Detection, Detections

# Import VLM implementations
from BOS_VLM.models.base_model import BaseVLM
from BOS_VLM.models.clip_model import CLIPModel
# Add more model imports as needed

class VLMNode:
    """BOS node for Visual Language Models."""
    # [Rest of implementation...]
