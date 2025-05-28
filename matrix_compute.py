import json
import numpy as np

def load_data(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)
    pixel_data = data[0]
    real_data = data[1]
    return pixel_data, real_data

def compute_scale_and_offset(pixel_data, real_data):
    pixel_points = []
    real_points = []

    for key in pixel_data:
        if key in real_data:
            px = pixel_data[key]['x']
            py = pixel_data[key]['y']
            rx = real_data[key]['x']
            ry = real_data[key]['y']
            pixel_points.append((px, py))
            real_points.append((rx, ry))

    pixel_points = np.array(pixel_points)
    real_points = np.array(real_points)

    # x-direction: solve for a, b in: pixel_x = a * real_x + b
    A_x = np.vstack([real_points[:, 0], np.ones(len(real_points))]).T
    a, b = np.linalg.lstsq(A_x, pixel_points[:, 0], rcond=None)[0]

    # y-direction: solve for c, d in: pixel_y = c * real_y + d
    A_y = np.vstack([real_points[:, 1], np.ones(len(real_points))]).T
    c, d = np.linalg.lstsq(A_y, pixel_points[:, 1], rcond=None)[0]

    return a, b, c, d

if __name__ == "__main__":
    json_file = "destination_map.json"  # 改成你的文件名
    pixel_data, real_data = load_data(json_file)
    a, b, c, d = compute_scale_and_offset(pixel_data, real_data)

    print("x_pixel = {:.4f} * x_real + {:.4f}".format(a, b))
    print("y_pixel = {:.4f} * y_real + {:.4f}".format(c, d))
