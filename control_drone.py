import sys
import os
import logging
import signal
import zmq
import math
import time
import scipy.io
import numpy as np

import simplejson
from simple_pid import PID

cmd = {
    "version": 1,
    "client_name": "N/A",
    "ctrl": {
        "roll": 0,
        "pitch": 0,
        "yaw": 0,
        "thrust": 0,
        "estop": False
    }
}

param = {
    "version": 1,
    "cmd": "set",
    "name": "flightmode.stabModeYaw",
    "value": "1".encode("utf-8")
}


context = zmq.Context()
client_conn = context.socket(zmq.PUSH)
client_conn.connect("tcp://127.0.0.1:1212")

realsense_connect = context.socket(zmq.PULL)
realsense_connect.connect("tcp://127.0.0.1:7777")

setpoint_conn = context.socket(zmq.PULL)
setpoint_conn.connect("tcp://127.0.0.1:5124")

set_param = context.socket(zmq.REQ)
set_param.connect("tcp://127.0.0.1:1213")

setpoint_viz = context.socket(zmq.PUSH)
setpoint_viz.bind("tcp://127.0.0.1:7778")

pid_x_sp = 0
pid_y_sp = 0
pid_z_sp = -0.45
pid_vel_sp = 0

setpoint_visual = {
    "vis": {
        "x": 0,
        "y": 0,
        "z": 0
    }
}

pid_x = PID(12, 8, 5, setpoint=pid_x_sp) # 12, 6, 5# 15 ,5 ,5 proportional 30, 10, 15,
pid_y = PID(30, 10, 15, setpoint=pid_y_sp)  # 2, 1, 0.5 hover icin stabil 30, 10, 10,
pid_z = PID(0.03, 0.03, 0.075, setpoint=pid_z_sp) # 0.1, 0.05, 0.03 hover
pid_verticalpos = PID(0.02, 0, 0.05, setpoint=pid_vel_sp) # 0.05, 0.02, 0
pid_yaw = PID(1, 0, 0, setpoint=0)

# Flight dynamics
# Pitch + --> y -
# Roll + --> x +

delta = 0.025

pid_z.sample_time = delta
pid_y.sample_time = delta
pid_x.sample_time = delta
pid_yaw.sample_time = delta
pid_z.proportional_on_measurement = False
pid_y.proportional_on_measurement = False
pid_x.proportional_on_measurement = False
pid_verticalpos.proportional_on_measurement = False
pid_z.output_limits = (-0.005, 0.035)
pid_y.output_limits = (-10, 10)
pid_yaw.output_limits = (-20, 20)
pid_verticalpos.output_limits = (-0.02, 0.02)

prev_thrust = 0
prev_z = 0

samples = []
thrust_data = []
z_pos = []
pitch_data = []
y_pos = []
roll_data = []
x_pos = []

pid_x_sp_data = []
pid_y_sp_data = []
pid_z_sp_data = []

px_data = []
ix_data = []
dx_data = []

py_data = []
iy_data = []
dy_data = []

pz_data = []
iz_data = []
dz_data = []

curr_vel_data = []
vel_data = []
pvel_data = []
ivel_data = []
dvel_data = []

feedforward_thrust_data = []

counter = 1

client_conn.send_json(cmd, zmq.NOBLOCK)

while True:
    try:
        data = realsense_connect.recv_json(zmq.NOBLOCK)
        # print("Got data")
        x = data["data"]["x"]
        y = data["data"]["y"]
        z = data["data"]["z"]
        yaw_angle = data["data"]["yaw_angle"]
        detected = data["data"]["detected"]
        feedforward_thrust = 0.59
        print(x)

        setpoint_visual["vis"]["x"] = pid_x.setpoint
        setpoint_visual["vis"]["y"] = pid_y.setpoint
        setpoint_visual["vis"]["z"] = pid_z.setpoint

        setpoint_viz.send_json(setpoint_visual, zmq.NOBLOCK)

        thrust = pid_z(z) + feedforward_thrust

        curr_velocity = (z - prev_z)/delta
        vel = pid_verticalpos(curr_velocity)

        pvel, ivel, dvel = pid_verticalpos.components

        #print(thrust + vel)

        pz, iz, dz = pid_z.components
        # print(thrust)

        pitch = pid_y(y) + 0.25
        py, iy, dy = pid_y.components
        # print(-pitch)

        roll = pid_x(x) - 0.25
        px, ix, dx = pid_x.components
        # print(roll)

        yaw_vel = pid_yaw(yaw_angle)

        cmd["ctrl"]["roll"] = roll
        cmd["ctrl"]["pitch"] = -pitch
        cmd["ctrl"]["thrust"] = (thrust) * 100
        cmd["ctrl"]["yaw"] = 0 # yaw_vel
        client_conn.send_json(cmd, zmq.NOBLOCK)

        prev_z = z
        prev_thrust = thrust

        samples.append(counter)

        thrust_data.append(thrust*100)
        z_pos.append(z)

        pitch_data.append(-pitch)
        y_pos.append(y)

        roll_data.append(roll)
        x_pos.append(x)

        px_data.append(px)
        ix_data.append(ix)
        dx_data.append(dx)

        py_data.append(py)
        iy_data.append(iy)
        dy_data.append(dy)

        pz_data.append(pz)
        iz_data.append(iz)
        dz_data.append(dz)

        curr_vel_data.append(curr_velocity)
        vel_data.append(vel)
        pvel_data.append(pvel)
        ivel_data.append(ivel)
        dvel_data.append(dvel)

        feedforward_thrust_data = feedforward_thrust
        pid_x_sp_data = pid_x_sp
        pid_y_sp_data = pid_y_sp
        pid_z_sp_data = pid_z_sp
        pid_vel_sp_data = pid_vel_sp

        counter = counter + 1

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/samples.mat', mdict={'samples': samples})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/thrust_data.mat', mdict={'thrust_data': thrust_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/z_pos.mat', mdict={'z_pos': z_pos})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/roll_data.mat', mdict={'roll_data': roll_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/x_pos.mat', mdict={'x_pos': x_pos})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pitch_data.mat', mdict={'pitch_data': pitch_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/y_pos.mat', mdict={'y_pos': y_pos})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/px_data.mat', mdict={'px_data': px_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/ix_data.mat', mdict={'ix_data': ix_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/dx_data.mat', mdict={'dx_data': dx_data})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/py_data.mat', mdict={'py_data': py_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/iy_data.mat', mdict={'iy_data': iy_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/dy_data.mat', mdict={'dy_data': dy_data})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pz_data.mat', mdict={'pz_data': pz_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/iz_data.mat', mdict={'iz_data': iz_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/dz_data.mat', mdict={'dz_data': dz_data})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/curr_vel_data.mat', mdict={'curr_vel_data': curr_vel_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/vel_data.mat', mdict={'vel_data': vel_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pvel_data.mat', mdict={'pvel_data': pvel_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/ivel_data.mat', mdict={'ivel_data': ivel_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/dvel_data.mat', mdict={'dvel_data': dvel_data})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/feedforward_thrust_data.mat', mdict={'feedforward_thrust_data': feedforward_thrust_data})

        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pid_x_sp_data.mat', mdict={'pid_x_sp_data': pid_x_sp_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pid_y_sp_data.mat', mdict={'pid_y_sp_data': pid_y_sp_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pid_z_sp_data.mat', mdict={'pid_z_sp_data': pid_z_sp_data})
        scipy.io.savemat('/home/control/PycharmProjects/ENcopter/data/pid_vel_sp_data.mat', mdict={'pid_vel_sp_data': pid_vel_sp_data})

    except zmq.error.ZMQError:
        pass
