#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import threading
import queue
from omni.msg import ballPoint, ballStraightPoint  # Import your custom messages

def preprocess_frame(frame):
    """
    Preprocess the frame for adaptive lighting using CLAHE.
    """
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply CLAHE to the V channel for better lighting adaptation
    h, s, v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v = clahe.apply(v)
    
    # Merge the adjusted V channel back to HSV
    hsv = cv2.merge((h, s, v))
    
    return hsv

def detect_ball(frame, lower_orange, upper_orange):
    """
    Detect an orange ball in the frame using color-based segmentation.
    Returns the ball's center (x, y), radius, and angle (if applicable).
    """
    # Preprocess the frame for adaptive lighting
    hsv = preprocess_frame(frame)

    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour (assumed to be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # Calculate the angle (for ballStraightPoint)
        if radius > 7.43:  # Only process if the ball is large enough
            ellipse = cv2.fitEllipse(largest_contour)
            angle = ellipse[2]  # Angle of the major axis
            return (int(x), int(y)), int(radius), angle, mask  # Return mask as well
    return None, None, None, mask  # Return mask even if no ball is detected

def init_kalman():
    """
    Initialize the Kalman Filter.
    """
    kalman = cv2.KalmanFilter(4, 2)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    return kalman

def track_ball(kalman, x, y):
    """
    Track the ball using Kalman Filter.
    """
    measurement = np.array([[np.float32(x)], [np.float32(y)]])
    kalman.correct(measurement)
    prediction = kalman.predict()
    return int(prediction[0]), int(prediction[1])

def capture_frames(cap, frame_queue):
    """
    Thread function to continuously capture frames from the camera.
    """
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame!")
            continue
        if frame_queue.full():
            frame_queue.get()  # Remove the oldest frame if the queue is full
        frame_queue.put(frame)

def detect_and_publish(frame_queue, pub_ball, pub_straight_ball, lower_orange, upper_orange, kalman):
    """
    Thread function to detect the ball and publish messages.
    """
    # trajectory = []  # List to store ball positions for trajectory (commented out)

    while not rospy.is_shutdown():
        if not frame_queue.empty():
            frame = frame_queue.get()

            # Detect the ball in the frame
            result = detect_ball(frame, lower_orange, upper_orange)

            if result[0] is not None:  # Check if a ball is detected
                (x, y), radius, angle, mask = result

                # Track the ballower_orange = [5, 150, 100], upper_orange = [15, 255, 255]l using Kalman Filter
                predicted_x, predicted_y = track_ball(kalman, x, y)

                # Add the current ball position to the trajectory (commented out)
                # trajectory.append((x, y))
                # if len(trajectory) > 50:  # Limit the trajectory length
                #     trajectory.pop(0)

                # Create a masked frame (only show the orange ball)
                masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

                # Draw a circle at the center of the ball on the masked frame
                cv2.circle(masked_frame, (x, y), 5, (0, 255, 0), -1)  # Green dot at the center

                # Draw a green outlined circle around the detected ball on the masked frame
                cv2.circle(masked_frame, (x, y), radius, (0, 255, 0), 2)  # Green outlined circle

                # Draw a line from the center of the frame to the center of the ball on the masked frame
                frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                cv2.line(masked_frame, frame_center, (x, y), (0, 255, 0), 2)

                # Draw the predicted position from Kalman Filter on the masked frame
                cv2.circle(masked_frame, (predicted_x, predicted_y), 5, (0, 0, 255), -1)  # Red dot for prediction

                # Create a copy of the original frame for ball detection visualization
                detection_frame = frame.copy()

                # Draw a circle at the center of the ball on the detection frame
                cv2.circle(detection_frame, (x, y), 5, (0, 255, 0), -1)  # Green dot at the center

                # Draw a green outlined circle around the detected ball on the detection frame
                cv2.circle(detection_frame, (x, y), radius, (0, 255, 0), 2)  # Green outlined circle

                # Draw a line from the center of the frame to the center of the ball on the detection frame
                cv2.line(detection_frame, frame_center, (x, y), (0, 255, 0), 2)

                # Draw the predicted position from Kalman Filter on the detection frame
                cv2.circle(detection_frame, (predicted_x, predicted_y), 5, (0, 0, 255), -1)  # Red dot for prediction

                # Publish the ballPoint message
                ball_msg = ballPoint()
                ball_msg.x = x
                ball_msg.y = ylower_orange = [5, 150, 100], upper_orange = [15, 255, 255]
                ball_msg.radius = radius
                pub_ball.publish(ball_msg)

                # Publish the ballStraightPoint message
                straight_ball_msg = ballStraightPoint()
                straight_ball_msg.distance = radius  # Example: Use radius as distance
                straight_ball_msg.degree = angle if angle is not None else 0.0
                pub_straight_ball.publish(straight_ball_msg)

                # Log the detected ball data
                rospy.loginfo(f"Detected ball: x={x}, y={y}, radius={radius}, angle={angle}")
            else:
                rospy.loginfo("No ball detected")
                _, _, _, mask = result
                masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
                detection_frame = frame.copy()  # Use a copy of the original frame for detection visualization

            # Display the original frame, masked frame, and frame with ball detection
            cv2.imshow("Original Frame", frame)
            cv2.imshow("Masked Frame", masked_frame)
            cv2.imshow("Frame with Ball Detection", detection_frame)

            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

def main():
    # Initialize the ROS node
    rospy.init_node('omni_camera_publisher', anonymous=True)

    # Create publishers for the custom messages
    pub_ball = rospy.Publisher('ball_data', ballPoint, queue_size=10)
    pub_straight_ball = rospy.Publisher('ball_straight_data', ballStraightPoint, queue_size=10)

    # Initialize the camera
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Enable auto focus and auto exposure
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable auto focus
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Enable auto exposure

    if not cap.isOpened():
        rospy.logerr("Could not open camera!")
        return

    # Define the range for the orange color in HSV
    lower_orange = np.array([5, 150, 100])  # Lower bound for orange in HSV
    upper_orange = np.array([15, 255, 255])  # Upper bound for orange in HSV

    # Initialize Kalman Filter
    kalman = init_kalman()

    # Create a queue to share frames between threads
    frame_queue = queue.Queue(maxsize=10)

    # Start the frame capture thread
    capture_thread = threading.Thread(target=capture_frames, args=(cap, frame_queue))
    capture_thread.start()

    # Start the detection and publishing thread
    detect_thread = threading.Thread(target=detect_and_publish, args=(frame_queue, pub_ball, pub_straight_ball, lower_orange, upper_orange, kalman))
    detect_thread.start()

    # Set the frame rate
    rate = rospy.Rate(10)  # 10 Hz

    # Wait for threads to finish
    capture_thread.join()
    detect_thread.join()

    # Release the camera and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass