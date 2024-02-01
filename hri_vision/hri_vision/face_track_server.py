import time
import threading

from hri_interfaces.action import VideoTracker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image 

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.timer import Rate,Timer

from collections import defaultdict

import cv2
import numpy as np

from ultralytics import YOLO
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


class VideoTrackerOb(Node):

    def __init__(self):
        super().__init__('video_tracker_server_node')

        self._action_server = ActionServer(
            self,
            VideoTracker,
            'face_tracker',
            execute_callback = self.execute_callback,
            callback_group = ReentrantCallbackGroup(),
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback)

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 1 messages.
        self.img_sub = self.create_subscription(
            Image, 
            #'video_frames', 
            '/image_raw',
            self.image_listener_callback, 
            1)
        self.img_sub # prevent unused variable warning
      
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        # Load the YOLOv8 model
        self.model = YOLO('/home/toto/Gdrive/uni/robocup/robocup_ws/src/hri/hri_vision/hri_vision/models/yolov8n-face.pt')

        # Store the track history
        self.track_history = defaultdict(lambda: [])

        self.received_frame=np.zeros((480,640,3), np.uint8)

        self.frame = np.zeros((480,640,3), np.uint8)

        self._loop_rate = self.create_rate( 1.0 , self.get_clock()) # Hz

        self.get_logger().info('node initialized')

        self.newdata = False

    def image_listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().debug('Receiving video frame')
        #self.newdata = True
        self.received_frame=data


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        feedback_msg = VideoTracker.Feedback()

        center = Point()


        # Loop through the video frames
        while rclpy.ok():
            
            #if self.newdata:
                #self.newdata=False;
                # Convert ROS Image message to OpenCV image
                current_frame = self.br.imgmsg_to_cv2(self.received_frame)
                #print('output dtype      : {}'.format(current_frame.dtype))
                #print('output shape      : {}'.format(current_frame.shape))
                #print('output encoding      : {}'.format(current_frame.tostring()))

                img = current_frame.reshape(480, 640, 2)
                rgb = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
                rgb = cv2.rotate(rgb, cv2.ROTATE_180)
                self.frame = rgb
                
                # Run YOLOv8 tracking on the frame, persisting tracks between frames
                results = self.model.track(self.frame, persist=True, tracker="bytetrack.yaml")

                # Get the boxes and track IDs
                boxes = results[0].boxes.xywh.cuda()
                
                #boxes = results[0].boxes.xywh.cuda()
                #track_ids = results[0].boxes.id.int().cuda().tolist()


                try:
                    track_ids = results[0].boxes.id.int().cuda().tolist()

                    # Plot the tracks
                    for box, track_id in zip(boxes, track_ids):
                        x, y, w, h = box
                        #track = self.track_history[track_id]
                        #track.append((float(x), float(y)))  # x, y center point
                        #if len(track) > 30:  # retain 90 tracks for 90 frames
                            #track.pop(0)

                        # Draw the tracking lines
                        #points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                        #cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

                        center.x = float(x)
                        center.y = float(y)

                        feedback_msg.center = center

                        # Publish the feedback
                        goal_handle.publish_feedback(feedback_msg)

                        #annotated_frame = results[0].plot()
                        #cv2.imshow("YOLOv8 Tracking", annotated_frameed_frame)
                            

                except AttributeError:
                    self.get_logger().warning('no face detected')
                    center.x = -1.0
                    center.y = -1.0
                    feedback_msg.center = center
                    goal_handle.publish_feedback(feedback_msg)
                    
                except:
                    self.get_logger().error('something else get wrong')

                self._loop_rate.sleep()    
                # Visualize the results on the frame
                       

        goal_handle.succeed()    

        # Populate result message
        result = VideoTracker.Result()
        result.success = True

        self.get_logger().info('Returning result')

        return result


def main(args=None):
    rclpy.init(args=args)

    video_tracker_server = VideoTrackerOb()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(video_tracker_server, executor=executor)

    video_tracker_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()