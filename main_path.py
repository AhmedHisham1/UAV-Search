import numpy as np
import sys, os
import cv2
from time import time
import tensorflow as tf
from yolov3_tf2.models import (YoloV3, YoloV3Tiny)
import argparse
from PyQt5.QtWebEngineWidgets import QWebEngineView     # ImportError: QtWebEngineWidgets must be imported before a QCoreApplication instance is created

detector = 'yolo-tiny'   # detector: 'yolo', 'yolo-tiny', 'ssd'

if detector == 'ssd':
    sys.path.append("..")
    from utils import label_map_util
    import tensorflow.compat.v1 as tf
    tf.disable_v2_behavior()

from gazebo_camera import Video
video = Video()
from cv_utils import coordinates_plot, draw_outputs, keep_person_only, localize_person

from mavsdk import System, MissionItem, OffboardError, PositionNedYaw
from mavsdk_utils import connect_sitl, get_relative_altitude, get_euler_angles, get_lat_lon, land, square_mission, square_mission_offboard, path_mission
import asyncio

def transform_images_for_yolo(x_train, size):
    x_train = tf.image.resize(x_train, (size, size))
    x_train = x_train / 255
    return x_train

def initialize_ssd_detector():
    MODEL_NAME = 'trt_ssdlite'
    PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'
    PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
    return detection_graph, category_index

async def mission(show_frames=True):
    times = []
    person_lats_all = []
    person_lons_all = []

    if detector == 'ssd':
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
    else:
        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        if len(physical_devices) > 0:
            tf.config.experimental.set_memory_growth(physical_devices[0], True)
    with tf.device('/gpu:0'):
        if detector == 'yolo' or detector == 'yolo-tiny':
            # initiallizing the detecor
            coco_classes = './yolo_data/coco.names'    # Path to classes file
            num_classes = 80    # number of classes in the model
            size = 416    # resize images to
            # size = 320
            if detector == 'yolo-tiny':
                print('Detector: YOLO-Tiny')
                weights = './checkpoints/yolov3-tiny.tf'    # path to weights file
                yolo = YoloV3Tiny(classes=num_classes)
            else:
                weights = './checkpoints/yolov3.tf'    # path to weights file
                yolo = YoloV3(classes=num_classes)
            yolo.load_weights(weights)
            print('weights loaded')
            class_names = [c.strip() for c in open(coco_classes).readlines()]
            print('classes loaded')
            try:
                # goal_loc = [38.16210, -122.45653]
                # goal_loc = [38.160869, -122.452910]
                goal_loc = [38.158726, -122.452053]
                drone, home_lat, home_lon = await path_mission(goal_loc, mission_alt=50, mission_spd=50, RTL_alt=10, 
                                                                                                         CAM_pitch=-90, 
                                                                                                         CAM_yaw=-90, VTOL=True)
            except:
                print("\nMission couldn't be started!\n\n")
            while True:
                if not video.frame_available():
                    continue

                img = video.frame()
                image_tf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
                image_tf = tf.expand_dims(image_tf, 0)
                image_tf = transform_images_for_yolo(image_tf, size)
                t1 = time()
                boxes, scores, classes, nums = yolo.predict(image_tf)
                times.append(time() - t1)
                times = times[-20:]
                boxes_squeezed,scores_squeezed,classes_squeezed,nums = keep_person_only(np.squeeze(boxes), 
                                                                                        np.squeeze(scores), 
                                                                                        np.squeeze(classes).astype(np.int32), 
                                                                                        confidence=0.2,
                                                                                        detector=detector)
                if nums > 0:    # A person was detected
                    print(f'yolo nums: {nums}')
                    print(f'score: {scores_squeezed}')
                    try:
                        current_altitude = await get_relative_altitude(drone)
                        _,_,current_yaw = await get_euler_angles(drone)
                        current_lat, current_lon = await get_lat_lon(drone)
                        person_lats, person_lons = localize_person(img=img, boxes=boxes_squeezed, 
                                                    nums=nums, alt=current_altitude, yaw=current_yaw, 
                                                    drone_lat=current_lat, drone_lon=current_lon)
                        # await drone.mission.pause_mission()
                        # await land(drone)
                        # print('Mission is paused.. \n\tA person is detected!')
                        person_lats_all.extend(person_lats)
                        person_lons_all.extend(person_lons)
                    except:
                        pass
                # Display Output
                if show_frames:
                    img = draw_outputs(img, (boxes_squeezed,
                                             scores_squeezed, 
                                             classes_squeezed, 
                                             nums), class_names, detector=detector)
                    img = cv2.putText(img, "Time: {:,.2f}ms | FPS: {:.2f}fps".format(times[-1]*1000, 1/times[-1]),
                                            (0, 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 255), 2)
                    cv2.imshow('object detection', img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        cv2.destroyAllWindows()
                        if len(person_lats_all):
                            coordinates_plot(person_lat=np.mean(person_lats_all), person_lon=np.mean(person_lons_all), 
                                            home_lat=home_lat, home_lon=home_lon)
                        break

        elif detector == 'ssd':
            detection_graph, category_index = initialize_ssd_detector()
            with detection_graph.as_default():
                with tf.Session(graph=detection_graph, config=config) as sess:
                    # Starting the pre-planned mission after starting the tf session to eliminate the delayed start of the detection
                    goal_loc = [38.16210, -122.45653]
                    drone, home_lat, home_lon = await path_mission(goal_loc, mission_alt=10, mission_spd=50, RTL_alt=10, 
                                                                                                             CAM_pitch=-90, 
                                                                                                             CAM_yaw=-90, VTOL=True)
                    # Detection loop
                    while True:
                        if not video.frame_available():
                            continue

                        image_np = video.frame()
                        image_np_expanded = np.expand_dims(image_np, axis=0)
                        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                        scores = detection_graph.get_tensor_by_name('detection_scores:0')
                        classes = detection_graph.get_tensor_by_name('detection_classes:0')
                        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                        # Actual detection.
                        t1 = time()
                        (boxes, scores, classes, num_detections) = sess.run(
                        [boxes, scores, classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})
                        times.append(time() - t1)
                        times = times[-20:]
                        boxes_squeezed,scores_squeezed,classes_squeezed,nums = keep_person_only(np.squeeze(boxes), 
                                                                                                np.squeeze(scores), 
                                                                                                np.squeeze(classes).astype(np.int32), 
                                                                                                confidence=0.5)
                        if nums > 0:    # A person was detected
                            try:
                                current_altitude = await get_relative_altitude(drone)
                                _,_,current_yaw = await get_euler_angles(drone)
                                current_lat, current_lon = await get_lat_lon(drone)
                                person_lats, person_lons = localize_person(img=image_np, boxes=boxes_squeezed, 
                                                            nums=nums, alt=current_altitude, yaw=current_yaw, 
                                                            drone_lat=current_lat, drone_lon=current_lon)
                                # await drone.mission.pause_mission()
                                # await land(drone)
                                # print('Mission is paused.. \n\tA person is detected!')
                                person_lats_all.extend(person_lats)
                                person_lons_all.extend(person_lons)
                            except:
                                pass
                        # Display output
                        if show_frames:
                            # Visualization of the results of a detection.
                            image_np = draw_outputs(image_np, (boxes_squeezed,
                                                        scores_squeezed, 
                                                        classes_squeezed, 
                                                        nums), category_index)
                            
                            image_np = cv2.putText(image_np, "Time: {:,.2f}ms | FPS: {:.2f}fps".format(times[-1]*1000, 1/times[-1]),
                                                (0, 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 255), 2)
                            
                            # cv2.imshow('object detection', cv2.resize(image_np, (800, 600)))
                            cv2.imshow('object detection', image_np)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                cv2.destroyAllWindows()
                                if len(person_lats_all):
                                    coordinates_plot(person_lat=np.mean(person_lats_all), person_lon=np.mean(person_lons_all), 
                                                    home_lat=home_lat, home_lon=home_lon)
                                break
        else:
            print('select a valid detector!')

    
        


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(mission())
    