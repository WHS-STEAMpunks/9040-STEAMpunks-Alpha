ImageRecognition.java
 - Separate class for TensorFlow Lite Image Recognition
 - Uses a tflite model to recognize the position of the team prop (Left, Middle, or Right) through a Bitmap, passed in through classify(Bitmap image, int imageRotation)
 - Uses BitmapFactory to convert PNG images passed in by the OpMode to Bitmaps
 - Has functionality to analyze inference time

Radium.java
 - Test OpMode for Autonomous (only prints info to telemetry, not game-ready)
 - Uses FTC VisionPortal to save PNG images from the webcam onto the Control Hub
 - Runs Image Recognition using ImageRecognition.java 
