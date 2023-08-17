#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import numpy as np

def draw_arrow(image, steering_angle, speed, max_speed=3000):
    height, width, _ = image.shape

    # Coordinates of the arrow origin
    org_x = 320
    org_y = 470

    # Defining max end_x offset for left and right turns
    max_offset = int(width * 0.5)

    # Scaling steering angle and calculating end_x
    angle_scaled = int(steering_angle * max_offset * 2) - max_offset

    # Scaling speed to the max_height (org_y - 200)
    max_height = abs(org_y - 200)
    speed_scaled = int(speed / max_speed * max_height)

    # Calculating the end coordinates of the arrow
    end_x = org_x + angle_scaled
    end_y = org_y - speed_scaled

    # Drawing the arrow
    arrow_color = (0, 255, 0)
    arrow_thickness = 3
    img = cv2.arrowedLine(image, (org_x, org_y), (end_x, end_y), arrow_color, arrow_thickness)
    # img = cv2.arrowedLine(image, (org_x, org_y), (end_x, end_y), arrow_color, arrow_thickness)

    return img


if __name__ == '__main__':               # 해당 Python 코드를 직접 실행할 때만 동작하도록 하는 부분
    pass

