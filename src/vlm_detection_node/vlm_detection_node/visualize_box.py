import cv2


def visualize_detections(image, detection_result):
    start_coords = detection_result.get("start_coordinates", [])[0]
    end_coords = detection_result.get("end_coordinates", [])[0]
    start_label = detection_result.get("start", "Start")
    end_label = detection_result.get("end", "End")

    # Draw bounding box for the start object
    cv2.rectangle(image, (start_coords[0], start_coords[1]), (start_coords[2], start_coords[3]), (0, 255, 0), 2)
    cv2.putText(image, start_label, (start_coords[0], start_coords[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Draw bounding box for the end object
    cv2.rectangle(image, (end_coords[0], end_coords[1]), (end_coords[2], end_coords[3]), (0, 0, 255), 2)
    cv2.putText(image, end_label, (end_coords[0], end_coords[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Display
    cv2.imshow("Detecte Objects", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
