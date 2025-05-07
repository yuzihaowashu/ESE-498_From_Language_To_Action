 # fastapi_model_server.py
from fastapi import FastAPI, File, UploadFile, Form
from fastapi.middleware.cors import CORSMiddleware
import numpy as np
import cv2
import torch
from torchvision.transforms import ToTensor
from PIL import Image
from io import BytesIO
from sklearn.decomposition import PCA
from efficient_sam.build_efficient_sam import build_efficient_sam_vitt

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"])

# === Load model once ===
model = build_efficient_sam_vitt()
model.eval()

# === Utilities ===
def run_ours_box_or_points(img, pts_sampled, pts_labels, model):
    img_tensor = ToTensor()(img)
    pts_sampled = torch.reshape(torch.tensor(pts_sampled), [1, 1, -1, 2])
    pts_labels = torch.reshape(torch.tensor(pts_labels), [1, 1, -1])
    predicted_logits, predicted_iou = model(img_tensor[None, ...], pts_sampled, pts_labels)

    sorted_ids = torch.argsort(predicted_iou, dim=-1, descending=True)
    predicted_logits = torch.take_along_dim(predicted_logits, sorted_ids[..., None, None], dim=2)
    binary_mask = torch.ge(predicted_logits[0, 0, 0, :, :], 0).cpu().numpy()
    return binary_mask

def estimate_grasp_direction_from_mask(binary_mask):
    coords = np.argwhere(binary_mask > 0)
    if coords.shape[0] < 2:
        raise ValueError("Not enough points for PCA")
    coords = coords[:, [1, 0]]
    pca = PCA(n_components=2)
    pca.fit(coords)
    angle = np.arctan2(pca.components_[0][1], pca.components_[0][0])
    return angle, pca.mean_, pca.components_

# === API Endpoint ===
@app.post("/process/")
async def process_image(
    file: UploadFile = File(...),
    xmin: int = Form(...),
    ymin: int = Form(...),
    xmax: int = Form(...),
    ymax: int = Form(...)
):
    image_data = await file.read()
    image = Image.open(BytesIO(image_data)).convert("RGB")
    
    # Step 1: Convert to NumPy
    np_img = np.array(image)

    # Step 2: Build input point/label
    input_point = np.array([[xmin, ymin], [xmax, ymax]])
    input_label = np.array([2, 3])

    # Step 3: Run segmentation
    binary_mask = run_ours_box_or_points(image, input_point, input_label, model)

    # Step 4: Run PCA
    angle, center, components = estimate_grasp_direction_from_mask(binary_mask)
    print(angle)
    return {
        "angle_deg": float(np.degrees(angle)),
        "center": center.tolist(),
        "x_axis": components[0].tolist(),
        "y_axis": components[1].tolist()
    }