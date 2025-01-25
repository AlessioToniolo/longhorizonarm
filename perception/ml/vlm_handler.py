import cv2
from perception.camera.realsense_interface import RealSenseInterface
from perception.ml.vlm_model import VLMModel

class VLMHandler:
    def __init__(self):
        self.realsense = RealSenseInterface()
        self.vlm_model = VLMModel()

    def process_frame(self):
        frame = self.realsense.get_frame()
        if frame is not None:
            # Preprocess the frame if needed
            processed_frame = self.preprocess_frame(frame)
            # Feed the frame into the VLM model
            results = self.vlm_model.predict(processed_frame)
            return frame, results
        return None, None

    def preprocess_frame(self, frame):
        # Example preprocessing: convert to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return rgb_frame

    def stop(self):
        self.realsense.stop()