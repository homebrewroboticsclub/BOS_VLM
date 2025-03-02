#!/usr/bin/env python3
"""
Example script demonstrating how to use the BOS VLM package.
"""

import brew
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from BOS_VLM.msg import VLMQuery, VLMResponse, Detections

# Class to handle the VLM client
class VLMClient:
    def __init__(self):
        # Initialize BOS node
        brew.init_node('vlm_client', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Set up publishers
        self.query_pub = brew.Publisher('/vlm/query', VLMQuery, queue_size=10)
        
        # Set up subscribers
        self.response_sub = brew.Subscriber(
            '/vlm/response', 
            VLMResponse, 
            self.response_callback, 
            queue_size=10
        )
        
        self.detections_sub = brew.Subscriber(
            '/vlm/detections', 
            Detections, 
            self.detections_callback, 
            queue_size=10
        )
        
        # Store the latest image and responses
        self.latest_response = None
        self.latest_detections = None
        
        # Wait for publishers to be connected
        brew.sleep(1)
        brew.loginfo("VLM client initialized and ready")

    def response_callback(self, msg):
        """Handle incoming VLM responses."""
        self.latest_response = msg
        brew.loginfo(f"Received response: {msg.text} (confidence: {msg.confidence:.2f})")

    def detections_callback(self, msg):
        """Handle incoming object detections."""
        self.latest_detections = msg
        brew.loginfo(f"Received {len(msg.detections)} detections")
        
        for det in msg.detections:
            brew.loginfo(f"  {det.label}: ({det.x1:.2f}, {det.y1:.2f}, {det.x2:.2f}, {det.y2:.2f}) - {det.score:.2f}")

    def query_image(self, image, text=None, task="describe", model=None):
        """
        Send a query to the VLM node.
        
        Args:
            image: OpenCV image (RGB format)
            text: Optional text query or prompt
            task: Task to perform (describe, vqa, detection, embed, embed_text, similarity)
            model: Optional model to use
        """
        # Convert image to BOS format
        bos_image = self.bridge.cv2_to_imgmsg(image, "rgb8")
        
        # Create query message
        query = VLMQuery()
        query.header.stamp = brew.Time.now()
        query.image = bos_image
        query.text = text if text else ""
        query.task = task
        query.model = model if model else ""
        
        # Publish query
        self.query_pub.publish(query)
        brew.loginfo(f"Sent query with task: {task}")
        
        # Wait for response
        self.latest_response = None
        self.latest_detections = None
        
        timeout = brew.Time.now() + brew.Duration(5)
        while (self.latest_response is None and 
               self.latest_detections is None and 
               brew.Time.now() < timeout):
            brew.sleep(0.1)
            
        if self.latest_response is None and self.latest_detections is None:
            brew.logwarn("Timeout waiting for response")
            
        return self.latest_response, self.latest_detections

    def describe_image(self, image, model=None):
        """
        Get a description of an image.
        
        Args:
            image: OpenCV image (RGB format)
            model: Optional model to use
            
        Returns:
            Text description of the image
        """
        response, _ = self.query_image(image, task="describe", model=model)
        return response.text if response else None

    def answer_question(self, image, question, model=None):
        """
        Answer a question about an image.
        
        Args:
            image: OpenCV image (RGB format)
            question: Question about the image
            model: Optional model to use
            
        Returns:
            Answer to the question
        """
        response, _ = self.query_image(image, text=question, task="vqa", model=model)
        return response.text if response else None

    def detect_objects(self, image, queries=None, model=None):
        """
        Detect objects in an image.
        
        Args:
            image: OpenCV image (RGB format)
            queries: Optional list of object categories to detect
            model: Optional model to use
            
        Returns:
            List of detections
        """
        text = ','.join(queries) if queries else ""
        _, detections = self.query_image(image, text=text, task="detection", model=model)
        return detections.detections if detections else []

    def find_best_match(self, image, candidates, model=None):
        """
        Find the text that best matches the image.
        
        Args:
            image: OpenCV image (RGB format)
            candidates: List of text candidates
            model: Optional model to use
            
        Returns:
            Best matching text and confidence score
        """
        text = ','.join(candidates)
        response, _ = self.query_image(image, text=text, task="similarity", model=model)
        
        if response:
            return response.text, response.confidence
        else:
            return None, 0.0


def main():
    """Run a demo of the VLM client."""
    # Create client
    client = VLMClient()
    
    # Load a test image
    image_path = "test_image.jpg"
    try:
        # Load in BGR (OpenCV default)
        image = cv2.imread(image_path)
        # Convert to RGB for the model
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    except Exception as e:
        brew.logerr(f"Error loading image: {e}")
        return
    
    # Get a description of the image
    description = client.describe_image(image)
    brew.loginfo(f"Image description: {description}")
    
    # Ask a question about the image
    answer = client.answer_question(image, "What objects are visible in the image?")
    brew.loginfo(f"Answer: {answer}")
    
    # Detect objects in the image
    detections = client.detect_objects(image, ["person", "car", "dog"])
    brew.loginfo(f"Detected {len(detections)} objects")
    
    # Find the best match from candidates
    candidates = [
        "A person in a park",
        "A car on the road",
        "A dog playing with a ball",
        "A cat sleeping on a couch"
    ]
    best_match, confidence = client.find_best_match(image, candidates)
    brew.loginfo(f"Best match: {best_match} (confidence: {confidence:.2f})")
    
    # Keep the node running
    brew.spin()


if __name__ == "__main__":
    try:
        main()
    except brew.BrewInterruptException:
        pass
