import cv2
import requests

def get_rotation_angle(image, xmin, ymin, xmax, ymax, server_url="http://xxx"):#"YOUR computer's ip /process/"):
    try: 
        _, img_encoded = cv2.imencode('.jpg', image)
        response = requests.post(
            server_url,
            files={"file": img_encoded.tobytes()},
            data={"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax}, 
            timeout=10
        )

        if response.status_code == 200:
            result = response.json() 
            return float(result.get("angle_deg"))
        else: 
            print(f"FastAPI error: {response.status_code} - {response.text}")
            return None
    
    except Exception as e:
        print(f"Failed to call FastAPI: {e}")
        return None