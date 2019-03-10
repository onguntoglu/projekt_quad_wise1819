import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import math
import transforms3d
import zmq
import time
import simplejson

print("Environment Ready")

context = zmq.Context()
socket_realsense = context.socket(zmq.PUSH)
socket_realsense.bind("tcp://127.0.0.1:7777")

setpoint_viz = context.socket(zmq.PULL)
setpoint_viz.connect("tcp://127.0.0.1:7778")

cmd = {
    "version": 1,
    "client_name": "N/A",
    "data": {
        "x": 0.1,
        "y": 0.1,
        "z": 0.0,
        "angle_vis": 0.0,
        "detected": False
    }
}

markerImage = np.zeros((400, 400, 1), dtype="uint8")
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_detection_parameters = cv2.aruco.DetectorParameters_create()
screenshot_cnt = 1
for id_num in range(5):
    markerImage = cv2.aruco.drawMarker(aruco_dict, id_num, 200, markerImage, 1)
    file_string = "aruco" + str(id_num) + ".png"
    plt.imsave(file_string, markerImage, cmap='gray')

# Setup:>
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)
side = 0.035

align_to = rs.stream.color
align = rs.align(align_to)

# drone starts at floor
fromTop = True

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 30.0, (848, 480))

try:
    while True:
        # Store next frameset for later processing:
        frameset = pipeline.wait_for_frames()

        # align frameset to color stream
        aligned_frames = align.process(frameset)

        aligned_color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame()

        color_image = np.asanyarray(aligned_color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        detected_aruco_markers = cv2.aruco.detectMarkers(color_image, aruco_dict)
        marker_image = cv2.aruco.drawDetectedMarkers(color_image.copy(), detected_aruco_markers[0], detected_aruco_markers[1])
        marker_image_depth = cv2.aruco.drawDetectedMarkers(depth_image.copy(), detected_aruco_markers[0], detected_aruco_markers[1])

        colorizer = rs.colorizer()
        depth_stream = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        extrin = aligned_depth_frame.profile.get_extrinsics_to(aligned_color_frame.profile)
        intrin = aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics()

        camera_matrix = np.array([[intrin.fx, 0, intrin.ppx], [0, intrin.fy, intrin.ppy], [0, 0, 1]])

        camera_rot = np.array([[extrin.rotation[0], extrin.rotation[1], extrin.rotation[2]],
            [extrin.rotation[3], extrin.rotation[4], extrin.rotation[5]],
            [extrin.rotation[6], extrin.rotation[7], extrin.rotation[8]]])

        camera_trans = np.array([[extrin.translation[0], extrin.translation[1], extrin.translation[2]]])

        estimated_pose_markers = cv2.aruco.estimatePoseSingleMarkers(detected_aruco_markers[0], side, camera_matrix, np.asanyarray(intrin.coeffs),
            np.asanyarray(extrin.rotation), np.asanyarray(extrin.translation))

        marker_image_pose = marker_image.copy()
        # crosshairs for 640, 480
        # marker_image_pose = cv2.line(marker_image_pose.copy(), (320, 0), (320, 480), (0, 255, 0), 2)
        # marker_image_pose = cv2.line(marker_image_pose.copy(), (0, 240), (640, 240), (0, 255, 0), 2)

        # crosshairs for 848, 480
        marker_image_pose = cv2.line(marker_image_pose.copy(), (424, 0), (424, 480), (0, 255, 0), 2)
        marker_image_pose = cv2.line(marker_image_pose.copy(), (0, 240), (848, 240), (0, 255, 0), 2)

        try:
            for i in range(len(detected_aruco_markers[1])):
                cv2.aruco.drawAxis(marker_image_pose, camera_matrix, np.asanyarray(intrin.coeffs), estimated_pose_markers[0][i],
                    estimated_pose_markers[1][i], 0.035)

                rot_mat_marker_extrin = cv2.Rodrigues(estimated_pose_markers[0][i])
                rot_mat_marker_extrin_inv = cv2.Rodrigues(estimated_pose_markers[0][i])
                decomp = cv2.RQDecomp3x3(rot_mat_marker_extrin[0])
                rot_mat_marker_intrin = np.matmul(decomp[5], decomp[4])
                rot_mat_marker_intrin = np.matmul(rot_mat_marker_intrin, decomp[3])

                tr = np.arccos((np.trace(rot_mat_marker_intrin[0:3]) - 1)/2)*180/math.pi

                X = np.dot(rot_mat_marker_extrin[0], np.array([side/2, 0, 0])) + estimated_pose_markers[1][i]
                Y = np.dot(rot_mat_marker_extrin[0], np.array([0, side/2, 0])) + estimated_pose_markers[1][i]
                m1 = np.dot(rot_mat_marker_extrin[0], np.array([side/2, side/2, 0])) + estimated_pose_markers[1][i]
                m4 = np.dot(rot_mat_marker_extrin[0], np.array([side/2, -side/2, 0])) + estimated_pose_markers[1][i]
                marker_mid_z = estimated_pose_markers[1][i]

                marker_mid_ip = cv2.projectPoints(estimated_pose_markers[1][i], (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                marker_mid_ip = tuple(marker_mid_ip[0].astype(int).reshape(1, -1)[0])

                marker_z_ip = cv2.projectPoints(marker_mid_z, (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                marker_z_ip = tuple(marker_z_ip[0].astype(int).reshape(1, -1)[0])

                X_image_point = cv2.projectPoints(X, (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                X_image_point = tuple(X_image_point[0].astype(int).reshape(1, -1)[0])

                Y_image_point = cv2.projectPoints(Y, (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                Y_image_point = tuple(Y_image_point[0].astype(int).reshape(1, -1)[0])

                m1_ip = cv2.projectPoints(m1, (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                m1_ip = tuple(m1_ip[0].astype(int).reshape(1, -1)[0])

                m4_ip = cv2.projectPoints(m4, (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                m4_ip = tuple(m4_ip[0].astype(int).reshape(1, -1)[0])

                try:
                    setpoint_viz_data = setpoint_viz.recv_json(flags=zmq.NOBLOCK)
                    sp_x = setpoint_viz_data["vis"]["x"]
                    sp_y = setpoint_viz_data["vis"]["y"]
                    sp_z = abs(setpoint_viz_data["vis"]["z"])

                    sp_array = [sp_x, sp_y, sp_z]
                    sp_array = np.asanyarray(sp_array).reshape(1, 3)

                    setpoint_visual = cv2.projectPoints(sp_array, (0, 0, 0), (0, 0, 0), camera_matrix, np.asanyarray(intrin.coeffs))
                    setpoint_visual = tuple(setpoint_visual[0].astype(int).reshape(1, -1)[0])
                    cv2.arrowedLine(marker_image_pose, marker_mid_ip, setpoint_visual, (0, 255, 255), 2)

                except zmq.error.ZMQError:
                    pass

                cv2.putText(marker_image_pose, 'FRONT', X_image_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                # cv2.putText(marker_image_pose, 'BOTTOM', X_image_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                if fromTop is False:
                    cv2.putText(marker_image_pose, 'RIGHT', Y_image_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(marker_image_pose, 'M1', m1_ip, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(marker_image_pose, 'M4', m4_ip, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                else:
                    cv2.putText(marker_image_pose, 'LEFT', Y_image_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(marker_image_pose, 'M4', m1_ip, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(marker_image_pose, 'M1', m4_ip, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                taitbr = transforms3d.euler.mat2euler(rot_mat_marker_intrin[0:3], 'rzyx')
                taitbr_roll = transforms3d.euler.mat2euler(rot_mat_marker_intrin[0:3], 'rzxy')
                taitbr = tuple(np.array(taitbr)*180/math.pi)
                taitbr_roll = tuple(np.array(taitbr_roll)*180/math.pi)

                taitbr_end = [taitbr[0], taitbr[1], taitbr_roll[1]]
                corners = detected_aruco_markers[0][0].tolist()[0]

                pos_x = estimated_pose_markers[1][i][0][0]
                pos_y = estimated_pose_markers[1][i][0][1]
                pos_z = estimated_pose_markers[1][i][0][2]
                # angle_vis = np.arctan2((corners[0][1]-corners[2][1]), (corners[2][0]-corners[0][0]))*180/math.pi + 45
                yaw_angle = taitbr_end[0]

                if yaw_angle < 180 and yaw_angle > -90:
                    yaw_angle = yaw_angle - 90
                else:
                    yaw_angle = yaw_angle + 270

                if fromTop is True:
                    yaw_angle = -yaw_angle # from top
                    taitbr_end = [yaw_angle, -taitbr[1], taitbr_roll[1]]

                pos_string = "x: {:1.4f}, y: {:1.4f}, z: {:1.4f}".format(pos_x, pos_y, pos_z)
                ypr_string = "Yaw: {:2.2f}, Pitch: {:2.2f}, Roll: {:2.2f}".format(yaw_angle, taitbr_end[1], taitbr_end[2])

                cv2.putText(marker_image_pose, pos_string, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(marker_image_pose, ypr_string, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                if pos_x and pos_y and pos_z and yaw_angle:
                    if fromTop is False:
                        cmd["data"]["x"] = pos_x
                        cmd["data"]["y"] = pos_y
                        cmd["data"]["z"] = pos_z
                        cmd["data"]["yaw_angle"] = yaw_angle
                        cmd["data"]["detected"] = True
                    else:
                        cmd["data"]["x"] = pos_x
                        cmd["data"]["y"] = pos_y
                        cmd["data"]["z"] = -pos_z
                        cmd["data"]["yaw_angle"] = yaw_angle
                        cmd["data"]["detected"] = True
                else:
                    cmd["data"]["x"] = 0
                    cmd["data"]["y"] = 0
                    cmd["data"]["z"] = 0
                    cmd["data"]["yaw_angle"] = 0
                    cmd["data"]["detected"] = False
                try:
                    if cmd["data"]["detected"] is True:
                        socket_realsense.send_json(cmd, zmq.NOBLOCK)
                except zmq.error.Again:
                    pass
                print(cmd)

        except TypeError:
            pass

        # use color_image for RGB stream, depth_stream for Depth Stream
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', marker_image_pose)
        # Video output
        out.write(marker_image_pose)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.waitKey(1) & 0xFF == ord('s'):
            scr_string = "screenshot_nn" + str(screenshot_cnt) + ".png"
            screenshot_cnt = screenshot_cnt + 1
            cv2.imwrite(scr_string, marker_image_pose)
            print("lalala")

finally:
    # out.release()
    pipeline.stop()
    cv2.destroyAllWindows()
