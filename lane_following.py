import numpy as np
import pid as PID
import cv2


def get_lane_center(img: np.ndarray, lanes: np.ndarray):
    """
    Takes a list of lanes and returns the center of the closest lane and its slope.

        Parameters:
            lanes (np.ndarray): The list of lanes to process.

        Returns:
            center_intercept (float): The horizontal intercept of the center of the closest lane.
            center_slope (float): The slope of the closest lane.
    """
    height, width, _ = img.shape
    center = width / 2
    min = 10000000000
    closest_lane = np.array([])
    center_info = []
    for lane in lanes:
        print(lane)
        # [[[x1,y1,x2,y2],slope,intercept],[[x1,y1,x2,y2],slope,intercept]]
        intercepts = [lane[0][2],lane[1][2]]
        print(f"intercepts: {intercepts}")
        for i in range(0, len(intercepts) - 1):
            if abs(intercepts[i]-center) < min:
                min = intercepts[i]
                closest_lane = lane
    print(f"closest_lane:{closest_lane}")
    if len(closest_lane)!=0:    
        midpoint1 = [(closest_lane[0][2] + closest_lane[1][2]) / 2, 0]
        # midpoint2 = [((closest_lane[0][2] + closest_lane[0][1]) + (closest_lane[1][2] + closest_lane[1][1]))/2, 1]
        center_slope = (closest_lane[0][1]+closest_lane[1][1])/2
        # center_slope = (midpoint1[1] - midpoint2[1]) / (midpoint1[0] - midpoint2[0])
        center_xintercept = midpoint1[0]
        # [[[[x1,y1,x2,y2],slope,intercept],[[x1,y1,x2,y2],slope,intercept]]]
        cv2.line(img, (int(center[0]),height), (round(center[0]+height/center[1]), 0), (255, 0, 0), 3)
        center_info = [center_xintercept, center_slope]

    return center_info


def recommend_direction(img: np.ndarray, center: float, slope: float, pid_horizontal):
    """
    Takes the center of the closest lane and its slope as inputs and returns a direction.

        Parameters:
            center (float): The center of the closest lane.
            slope (float): The slope of the closest lane.

        Returns:
            direction (str): left, right, forward
    """
    
    height, width, _ = img.shape
    camera_pov = width / 2
    dist = abs(camera_pov-center)
    percentx = (0.5 - dist / width) * 2
    lateral_power = pid_horizontal.update(percentx)
    print(f"lateral: {lateral_power}")
    
    if camera_pov < center and slope < 0:
        return "Right"
    elif camera_pov > center and slope > 0:
        return "Left"
    else:
        return "Straight"