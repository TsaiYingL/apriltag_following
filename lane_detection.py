import cv2, math
import numpy as np
import matplotlib.pyplot as plt


def detect_lines(
    img: np.ndarray,
    threshold1: int = 50,
    threshold2: int = 150,
    apertureSize: int = 3,
    minLineLength: int = 100,
    maxLineGap: int = 10,
) -> np.ndarray:
    """
    Takes an image and returns a list of detected lines.

        Parameters:
            img (np.ndarray): The image to process.
            threshold1 (int): The first threshold for the Canny edge detector (default: 50).
            threshold2 (int): The second threshold for the Canny edge detector (default: 150)
            apertureSize (int): The aperture size for the Sobel operator (default: 3)
            minLineLength (int): The minimum length of a line (default: 100)
            maxLineGap (int): The maximum gap between two points to be considered in the same line (default: 10)

        Returns:
            lines (np.ndarray)
    """
    img = cv2.GaussianBlur(img, (9,9), 0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _,bw = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
    #kernel = np.ones((5, 5), np.float32) / 25
    edges = cv2.Canny(bw, threshold1, threshold2, apertureSize=apertureSize)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, 100, minLineLength=minLineLength, maxLineGap=maxLineGap
    )
    return lines


def draw_lines(img: np.ndarray, lines: np.ndarray, color: tuple = (0, 255, 0)):
    """
    Takes an image and a list of lines and returns an image with the lines drawn on it.

        Parameters:
            img (str): The image to process.
            lines (np.ndarray): The list of lines to draw
            color (tuple): The color of the lines. Default: (0, 255, 0)
    """

    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), color, 2)


def get_slopes_intercepts(lines: np.ndarray):
    """
    Takes a list of lines and returns a list of slopes and a list of intercepts.

        Parameters:
            lines (np.ndarray)

        Returns:
            slopes (np.ndarray): The list of slopes
            intercepts (np.ndarray): The list of intercepts
    """

    slopes = (lines[:, :, 3] - lines[:, :, 1]) / (lines[:, :, 1] - lines[:, :, 0])
    # b = y - mx
    b = lines[:, :, 1] - slopes * lines[:, :, 0]
    intercepts = (np.zeros_like(slopes) - b) / slopes
    return slopes, intercepts


def detect_lanes(img: np.ndarray, lines: np.ndarray):
    """
    Takes a list of lines as an input and returns a list of lanes.

        Parameters:
            img (np.ndarray): The image, for comparing pixels.
            lines (np.ndarray): The list of lines to process.

        Returns:
            lanes (list): The list of lanes.
    """

    def avg_color(colors):
        colors = np.array(colors)
        r = sum(colors[:, 0])
        g = sum(colors[:, 1])
        b = sum(colors[:, 2])
        return (r / len(colors), g / len(colors), b / len(colors))

    def dist_color(c1, c2):
        """
        Returns distance between two colors.
        """

        (r1, g1, b1) = c1
        (r2, g2, b2) = c2
        return math.sqrt((r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2)

    def dist(l1, l2):
        return abs(l1[2] - l2[2])

    slopes, intercepts = get_slopes_intercepts(lines)
    # => ([x1, y1, x2, y2], slope, x-intercept) for every "line"
    # Let's sort the lines in order first
    sort = list(sorted(zip(lines, slopes, intercepts), key=lambda pair: pair[1]))

    cleaned = []
    for line in sort:
        can_add = True
        for clean in cleaned:
            if abs(clean[2] - line[2]) < 0.1:
                can_add = False

        if can_add:
            cleaned.append(line)

    i = 0
    height, width, _ = img.shape
    lanes = []
    if len(cleaned) == 2:
        return [cleaned]
    while i < len(cleaned) - 2:
        try:
            curr_line = cleaned[i]
            next_line = cleaned[i + 1]
            next_next_line = cleaned[i + 2]
            intercept1, intercept2 = curr_line[2], next_line[2]
            slope1, slope2 = curr_line[1], next_line[1]
            if dist(curr_line, next_line) < dist(next_line, next_next_line):
                lanes.append([curr_line, next_line])
                i += 2
                continue
            i += 1
        except:  
            i += 1
            continue
    return lanes


def draw_lanes(img: np.ndarray, lanes: list):
    """
    Takes an image and a list of lanes as inputs and returns an image with the lanes drawn on it.

        Parameters:
            img (np.ndarray): The image to process
            lanes (list)
    """

    random_color = lambda: list(np.random.random(size=3) * 256)
    for pair in lanes:
        color = random_color()
        for lane in pair:
            x1, y1, x2, y2 = lane[0][0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
