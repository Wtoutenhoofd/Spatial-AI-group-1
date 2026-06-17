# DetectorLive.py  (runs inside the venv)
# Polls the shared frame.jpg on disk and calls the Roboflow API every time a new
# frame arrives. Results are printed as JSON lines to stdout so the ROS node can
# read them through the subprocess pipe.
import time
import os
import json

from inference_sdk import InferenceHTTPClient

IMAGE_PATH = os.path.join("images", "frame.jpg")


class Detector:
    def __init__(self):
        # The Roboflow serverless endpoint — no local GPU needed, inference runs in the cloud.
        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key="XYZ"  # TODO: replace with your actual API key,
        )

    def run_inference(self, image_path: str):
        # Runs the "general-segmentation-api-3" Roboflow workflow, which detects
        # and segments objects of the specified class in the image.
        return self.client.run_workflow(
            workspace_name="spatial-ai-kdgzb",
            workflow_id="general-segmentation-api-3",
            images={"image": image_path},
            parameters={"classes": "Whiteboard"},
            use_cache=False,   # always want a fresh result
        )


def main():
    detector = Detector()
    last_mtime: float | None = None

    while True:
        # Wait until the ROS node has written at least one frame to disk.
        if not os.path.exists(IMAGE_PATH):
            time.sleep(0.05)
            continue

        mtime = os.path.getmtime(IMAGE_PATH)
        if mtime == last_mtime:
            # Same frame as last iteration — no new data to process yet.
            time.sleep(0.05)
            continue

        last_mtime = mtime

        try:
            result = detector.run_inference(IMAGE_PATH)
            # result[0] is the response for the first (and only) image sent to the workflow.
            predictions = result[0]["predictions"]["predictions"]

            for obj in predictions:
                # Build a slim dict with the fields downstream nodes actually use.
                msg = {
                    "class": obj["class"],
                    "confidence": float(obj["confidence"]),
                    "center_x": float(obj["x"]),
                    "center_y": float(obj["y"]),
                    "width": float(obj["width"]),
                    "height": float(obj["height"]),
                }
                # flush=True is required so the line reaches the ROS node immediately
                # rather than sitting in the OS pipe buffer.
                print(json.dumps(msg), flush=True)

        except Exception as e:
            # Route errors through the same pipe so the ROS node can log them.
            print(json.dumps({"error": str(e)}), flush=True)


if __name__ == "__main__":
    main()
