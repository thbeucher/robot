import time
import socket
import _XiaoRGEEK_SERVO_ as xservos

from _XiaoRGEEK_MOTOR_ import Robot_Direction

import robot_config.servos_config as sc

# Set up robot variables
go = Robot_Direction()
servos = xservos.XR_Servo()

# Set motors variables and functions
def move(steering_fn):
  steering_fn()
  time.sleep(0.1)
  go.stop()

# Set initial servos position
camera_vertical_angle = sc.camera_v_start
camera_horizontal_angle = sc.camera_h_start
claw_squeezer_angle = sc.claw_squeezer_start
claw_rotater_angle = sc.claw_rotater_start
claw_base_updown_angle = sc.claw_base_updown_start
claw_head_updown_angle = sc.claw_head_updown_start

servos.XiaoRGEEK_SetServoAngle(sc.camera_v_servos, camera_vertical_angle)
servos.XiaoRGEEK_SetServoAngle(sc.camera_h_servos, camera_horizontal_angle)
servos.XiaoRGEEK_SetServoAngle(sc.claw_squeezer_servos, claw_squeezer_angle)
servos.XiaoRGEEK_SetServoAngle(sc.claw_rotater_servos, claw_rotater_angle)
servos.XiaoRGEEK_SetServoAngle(sc.claw_base_updown_servos, claw_base_updown_angle)
servos.XiaoRGEEK_SetServoAngle(sc.claw_head_updown_servos, claw_head_updown_angle)

# Set up the network connection
SERVER_IP = '0.0.0.0'  # Listen on all network interfaces
PORT = 5050
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((SERVER_IP, PORT))
sock.listen()

# Accept incoming connections
conn, addr = sock.accept()
print('Connected by', addr)

# Loop until the connection is closed
while True:
  # Receive data from the client
  data = conn.recv(1024)
  if not data:
    break

  # Print the received data to the console
  key = data.decode('utf-8')
  print(key)

  if key == 'z':
    camera_vertical_angle += sc.camera_v_move
    camera_vertical_angle = min(camera_vertical_angle, sc.camera_v_max)
    servos.XiaoRGEEK_SetServoAngle(sc.camera_v_servos, camera_vertical_angle)
  elif key == 's':
    camera_vertical_angle -= sc.camera_v_move
    camera_vertical_angle = max(camera_vertical_angle, sc.camera_v_min)
    servos.XiaoRGEEK_SetServoAngle(sc.camera_v_servos, camera_vertical_angle)
  elif key == 'q':
    camera_horizontal_angle += sc.camera_h_move
    camera_horizontal_angle = min(camera_horizontal_angle, sc.camera_h_max)
    servos.XiaoRGEEK_SetServoAngle(sc.camera_h_servos, camera_horizontal_angle)
  elif key == 'd':
    camera_horizontal_angle -= sc.camera_h_move
    camera_horizontal_angle = max(camera_horizontal_angle, sc.camera_h_min)
    servos.XiaoRGEEK_SetServoAngle(sc.camera_h_servos, camera_horizontal_angle)
  elif key == 'up':
    move(go.right)
  elif key == 'down':
    move(go.left)
  elif key == 'left':
    move(go.forward)
  elif key == 'right':
    move(go.back)
  elif key == 'p':
    claw_squeezer_angle += sc.claw_squeezer_move
    claw_squeezer_angle = min(claw_squeezer_angle, sc.claw_squeezer_max)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_squeezer_servos, claw_squeezer_angle)
  elif key == 'o':
    claw_squeezer_angle -= sc.claw_squeezer_move
    claw_squeezer_angle = max(claw_squeezer_angle, sc.claw_squeezer_min)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_squeezer_servos, claw_squeezer_angle)
  elif key == 'm':
    claw_rotater_angle += sc.claw_rotater_move
    claw_rotater_angle = min(claw_rotater_angle, sc.claw_rotater_max)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_rotater_servos, claw_rotater_angle)
  elif key == 'l':
    claw_rotater_angle -= sc.claw_rotater_move
    claw_rotater_angle = max(claw_rotater_angle, sc.claw_rotater_min)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_rotater_servos, claw_rotater_angle)
  elif key == 'u':
    claw_base_updown_angle += sc.claw_base_updown_move
    claw_base_updown_angle = min(claw_base_updown_angle, sc.claw_base_updown_max)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_base_updown_servos, claw_base_updown_angle)
  elif key == 'i':
    claw_base_updown_angle -= sc.claw_base_updown_move
    claw_base_updown_angle = max(claw_base_updown_angle, sc.claw_base_updown_min)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_base_updown_servos, claw_base_updown_angle)
  elif key == 'j':
    claw_head_updown_angle += sc.claw_head_updown_move
    claw_head_updown_angle = min(claw_head_updown_angle, sc.claw_head_updown_max)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_head_updown_servos, claw_head_updown_angle)
  elif key == 'k':
    claw_head_updown_angle -= sc.claw_head_updown_move
    claw_head_updown_angle = max(claw_head_updown_angle, sc.claw_head_updown_min)
    servos.XiaoRGEEK_SetServoAngle(sc.claw_head_updown_servos, claw_head_updown_angle)

# Close the connection and socket
conn.close()
sock.close()
