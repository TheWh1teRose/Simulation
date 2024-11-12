import math
import cv2
import numpy as np
from pupil_apriltags import Detector
import matplotlib.pyplot as plt
import matplotlib

def track_apriltag_trajectory(video_path):
    # Initialize video capture from video file
    cap = cv2.VideoCapture(video_path)
    
    # Check if video opened successfully
    if not cap.isOpened():
        print("Error: Could not open video")
        return

    # Initialize AprilTag detector
    detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
    
    # Arrays to store trajectory
    x_positions = []
    y_positions = []
    e_distance = []

    end_point_x = 610   
    end_point_y = 395
    
    while True:
        # Read frame from video
        ret, frame = cap.read()
        if not ret:
            print("End of video or failed to read frame")
            break
            
        # Adjust brightness
        brightness_factor = 2.0
        adjusted_frame = cv2.convertScaleAbs(frame, alpha=brightness_factor, beta=0)
        
        # Convert adjusted frame to grayscale
        gray = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags in the frame
        results = detector.detect(gray)
        
        # Draw detection results and store positions
        for r in results:
            tag_id = r.tag_id
            center = tuple(int(c) for c in r.center)
            corners = r.corners.astype(int)

            if tag_id == 0:
                # Store positions
                x_positions.append(center[0])
                y_positions.append(center[1])
                e_distance.append(math.sqrt((center[0] - end_point_x)**2 + (center[1] - end_point_y)**2))
            
            # Draw the tag outline
            cv2.polylines(adjusted_frame, [corners], True, (0, 255, 0), 2)
            
            # Draw the tag center
            cv2.circle(adjusted_frame, center, 5, (0, 0, 255), -1)
            
            # Put the tag ID near the center
            cv2.putText(adjusted_frame, f"ID:{tag_id}", (center[0] - 10, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            print(f"Detected AprilTag ID: {tag_id} at position {center}")
        
        # Display the adjusted frame
        cv2.imshow('AprilTag Detection', adjusted_frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release everything
    cap.release()
    cv2.destroyAllWindows()
    
    # Plot trajectory with backend specification
    matplotlib.use('Agg')
    
    plt.figure(figsize=(10, 10))
    plt.plot(range(len(e_distance)), e_distance, 'b-', label='Tag Trajectory')
    plt.title('AprilTag Trajectory')
    plt.xlabel('X Position (pixels)')
    plt.ylabel('Y Position (pixels)')
    plt.legend()
    plt.grid(True)
    
    # Save the plot to a file (in case showing doesn't work)
    plt.savefig('trajectory_plot.png')

if __name__ == "__main__":
    video_path = "RealWorld/TrackVid.mp4"
    track_apriltag_trajectory(video_path)
