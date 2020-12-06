import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import pygame
import time
import cv2
import lcm
import os
from lcmtypes.image_t import image_t
from lcmtypes.simple_motor_command_t import simple_motor_command_t

os.system("bot-lcm-tunnel &")

camera_resolution = (640,480)
camera_framerate = 18

# Change this
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("Camera Feed")
screen = pygame.display.set_mode(camera_resolution)
camera = PiCamera()
camera.resolution = camera_resolution
camera.framerate = camera_framerate
rawCapture = PiRGBArray(camera, size=camera_resolution)

start_time = int(time.time() * 1e9)

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    image = frame.array

    # Comment this out if camera is mounted on top
    image = cv2.flip(image, -1)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    buff = cv2.imencode(".jpg", image)[1]

    image_packet = image_t()
    image_packet.utime = int(time.time() * 1e9)
    image_packet.size = buff.shape[0]
    image_packet.data = buff.tobytes()

    lc.publish("MBOT_IMAGE_STREAM", image_packet.encode())

    rawCapture.truncate(0)

    command = simple_motor_command_t()
    command.utime = int(time.time() * 1000000)
    command.angular_velocity = 0
    command.forward_velocity = 0.1
    key_input = pygame.key.get_pressed()
    if key_input[pygame.K_LEFT]:
        command.angular_velocity += ANGULAR_VEL_CONST
    if key_input[pygame.K_UP]:
        command.forward_velocity += FORWARD_VEL_CONST
    if key_input[pygame.K_RIGHT]:
        command.angular_velocity -= ANGULAR_VEL_CONST
    if key_input[pygame.K_DOWN]:
        command.forward_velocity -= FORWARD_VEL_CONST
    if key_input[pygame.K_q]:
        pygame.quit()
        cv2.destroyAllWindows()
        break
    lc.publish("MBOT_MOTOR_COMMAND_SIMPLE", command.encode())

