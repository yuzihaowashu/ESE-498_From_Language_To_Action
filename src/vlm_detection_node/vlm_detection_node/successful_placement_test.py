def is_successful_placement(start_coords, end_coords): 

    def compute_center(coords):
        x_min, y_min, x_max, y_max = coords
        return (x_min + x_max) / 2, (y_min + y_max) / 2

    def compute_distance(start_coords, end_coords): 
        x1, y1 = compute_center(start_coords)
        x2, y2 = compute_center(end_coords)
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

    # Step 1: check success
    center_x, center_y = compute_center(start_coords)
    # if the start object's center is within the bounding box of the end object
    within_x = end_coords[0] <= center_x <= end_coords[2]
    within_y = end_coords[1] <= center_y <= end_coords[3]

    success = within_x and within_y

    # Step 2: compute distance
    distance = compute_distance(start_coords, end_coords)
    return success, distance