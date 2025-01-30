from transformers import AutoModelForObjectDetection, DetrImageProcessor
import torch
import numpy as np

class VLMModel:
    def __init__(self):
        """
        Initialize the Vision Language Model for object detection.
        Uses DETR (DEtection TRansformer) model from Facebook.
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = AutoModelForObjectDetection.from_pretrained("facebook/detr-resnet-50").to(self.device)
        
    def predict(self, frame):
        """
        Make predictions on a single frame from camera stream.
        
        Args:
            frame: numpy array of shape (H, W, C) in BGR format
            
        Returns:
            list of dict: Each dict contains 'box' (coordinates), 'score', and 'label'
        """
        # Convert BGR to RGB
        frame_rgb = frame[..., ::-1]
        
        # Process image
        inputs = self.processor(images=frame_rgb, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        
        # Get predictions
        with torch.no_grad():
            outputs = self.model(**inputs)
        
        # Post-process predictions
        target_sizes = torch.tensor([frame.shape[:2]]).to(self.device)
        results = self.processor.post_process_object_detection(
            outputs, 
            target_sizes=target_sizes,
            threshold=0.5
        )[0]
        
        # Format results
        predictions = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            predictions.append({
                'box': box.cpu().numpy(),
                'score': score.cpu().numpy(),
                'label': self.model.config.id2label[label.item()]
            })
            
        return predictions