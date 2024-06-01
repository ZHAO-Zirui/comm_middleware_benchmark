import cv2
import numpy as np
import time


def generate_random_image(width: int, height: int) -> np.ndarray:
    return np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

def generate_timestamp() -> tuple:
    timestamp = time.time()
    secs = int(timestamp)
    nssecs = int((timestamp - secs) * 1e9)
    return secs, nssecs


if __name__ == "__main__":
    cv2.imshow("Random Image", generate_random_image(800, 600))
    cv2.waitKey(0)
    cv2.destroyAllWindows()