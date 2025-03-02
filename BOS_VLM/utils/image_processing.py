"""
Image processing utilities for BOS_VLM.
"""
import cv2
import numpy as np
from typing import Tuple

def resize_image(image: np.ndarray, max_size: int = 512) -> np.ndarray:
    """
    Resize an image while maintaining aspect ratio.
    
    Args:
        image: Input image as numpy array
        max_size: Maximum dimension (width or height)
        
    Returns:
        Resized image
    """
    height, width = image.shape[:2]
    
    # Calculate scale factor
    scale = min(max_size / width, max_size / height)
    
    # If image is smaller than max_size, don't resize
    if scale >= 1.0:
        return image
        
    # Calculate new dimensions
    new_width = int(width * scale)
    new_height = int(height * scale)
    
    # Resize image
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
    
    return resized

def normalize_image(image: np.ndarray) -> np.ndarray:
    """
    Normalize image pixel values to [0, 1].
    
    Args:
        image: Input image as numpy array
        
    Returns:
        Normalized image
    """
    return image.astype(np.float32) / 255.0

def center_crop(image: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
    """
    Center crop an image to the target size.
    
    Args:
        image: Input image as numpy array
        target_size: Target size as (width, height)
        
    Returns:
        Center cropped image
    """
    height, width = image.shape[:2]
    target_width, target_height = target_size
    
    # Calculate crop coordinates
    start_x = max(0, width // 2 - target_width // 2)
    start_y = max(0, height // 2 - target_height // 2)
    
    # Crop image
    cropped = image[start_y:start_y + target_height, start_x:start_x + target_width]
    
    return cropped
