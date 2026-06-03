import time
import os
import json

from inference_sdk import InferenceHTTPClient

IMAGE_PATH = os.path.join("images", "frame.jpg")
ROBOFLOW_API_KEY = os.environ["ROBOFLOW_API_KEY"]


class TextDetector:
    def __init__(self):
        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key=ROBOFLOW_API_KEY,
        )

    def run_inference(self, image_path: str):
        return self.client.run_workflow(
            workspace_name="spatial-ai-kdgzb",
            workflow_id="custom-workflow-2",
            images={"image": image_path},
            use_cache=False,
        )


def _parse_result(result: list) -> dict:
    if not result or not result[0]:
        return {"text": "", "words": []}

    data = result[0]

    # custom-workflow-2 returns OCR result under model_output
    if "model_output" in data:
        return {"text": data["model_output"], "words": []}

    # Fallback: plain ocr_text field
    if "ocr_text" in data:
        return {"text": data["ocr_text"], "words": []}

    # Workflow with a text-detection model returns bounding-box predictions
    # where each class label is the recognised word/character
    if "predictions" in data and "predictions" in data["predictions"]:
        words = []
        full_text_parts = []
        # Sort top-to-bottom, then left-to-right to reconstruct reading order
        preds = sorted(data["predictions"]["predictions"], key=lambda p: (p["y"], p["x"]))
        for pred in preds:
            text = pred.get("class", "")
            words.append({
                "text": text,
                "x": float(pred["x"]),
                "y": float(pred["y"]),
                "width": float(pred["width"]),
                "height": float(pred["height"]),
                "confidence": float(pred["confidence"]),
            })
            full_text_parts.append(text)
        return {"text": " ".join(full_text_parts), "words": words}

    return {"text": "", "words": []}


def main():
    detector = TextDetector()
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
            result = detector.run_inference(IMAGE_PATH)
            output = _parse_result(result)
            print(json.dumps(output), flush=True)

        except Exception as e:
            print(json.dumps({"error": str(e)}), flush=True)


if __name__ == "__main__":
    main()
