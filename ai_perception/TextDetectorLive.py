import time
import os
import json

import cv2
import pytesseract
from PIL import Image

IMAGE_PATH = os.path.join("images", "frame.jpg")

TESSERACT_CONFIG = "--psm 6 --oem 3"


def preprocess(image_path: str) -> Image.Image:
    img = cv2.imread(image_path)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Scale down to fixed height — tesseract works best with letters ~50-150px tall
    h, w = gray.shape
    target_h = 150
    gray = cv2.resize(gray, (int(w * target_h / h), target_h), interpolation=cv2.INTER_AREA)

    return Image.fromarray(gray)


def run_ocr(image_path: str) -> dict:
    img = preprocess(image_path)
    text = pytesseract.image_to_string(img, config=TESSERACT_CONFIG).strip()
    return {"text": text, "words": []}


def main():
    last_mtime: float | None = None

    while True:
        if not os.path.exists(IMAGE_PATH):
            time.sleep(0.05)
            continue

        mtime = os.path.getmtime(IMAGE_PATH)
        if mtime == last_mtime:
            time.sleep(0.05)
            continue

        last_mtime = mtime

        try:
            output = run_ocr(IMAGE_PATH)
            print(json.dumps(output), flush=True)
        except Exception as e:
            print(json.dumps({"error": str(e)}), flush=True)


if __name__ == "__main__":
    main()
