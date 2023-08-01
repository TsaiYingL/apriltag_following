import cv2
import numpy as np


def process_frame(
    frame,
    at_detector,
    camera_matrix=np.array([1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3, 3)),
    draw=False,
):
    """
    Detect the AprilTags.
    """
    x = []
    y = []
    camera_params = (
        camera_matrix[0, 0],
        camera_matrix[1, 1],
        camera_matrix[0, 2],
        camera_matrix[1, 2],
    )
    height, width, _ = frame.shape
    center = (int(width / 2), int(height / 2))
    if draw:
        cv2.line(
            frame, (int(width / 2), 0), (int(width / 2), int(height)), (255, 0, 0), 5
        )
        cv2.line(
            frame, (0, int(height / 2)), (int(width), int(height / 2)), (255, 0, 0), 5
        )
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # <-
    tags = at_detector.detect(frame, True, camera_params, 0.1)
    if draw:
        color_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
    for tag in tags:
        if draw:
            for idx in range(len(tag.corners)):
                # Drawing the lines and write text
                cv2.line(
                    color_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0),
                )
                cv2.putText(
                    color_img,
                    str(tag.tag_id),
                    org=(
                        tag.corners[0, 0].astype(int) + 10,
                        tag.corners[0, 1].astype(int) + 10,
                    ),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=2,
                    color=(0, 0, 255),
                )
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        x.append(np.interp(cX / width, 0, 0.5))
        y.append(np.interp(cY / height, 0, 0.5))
        if cX < center:
            x = -x
        if cY < center:
            y = -y
        if draw:
            dist = (cX - center[0], cY - center[1])
            cv2.circle(color_img, (cX, cY), 5, (0, 0, 255), -1)
            cv2.line(color_img, center, (int(cX), int(cY)), (0, 0, 255), 5)
    return x, y