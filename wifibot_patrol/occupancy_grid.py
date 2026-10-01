#!/usr/bin/env python3

import yaml
import cv2
import numpy as np
import os


def main():

    yaml_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/map.yaml"

    #
    # Load map.yaml
    #
    with open(yaml_file, "r") as file:

        config = yaml.safe_load(file)

    image_path = os.path.join(
        os.path.dirname(yaml_file),
        config["image"]
    )

    #
    # Load map.pgm
    #
    map_img = cv2.imread(
        image_path,
        cv2.IMREAD_GRAYSCALE
    )

    if map_img is None:

        print("ERROR: map not loaded")

        return

    #
    # Occupancy Grid
    #
    # obstacle = 1
    # free     = 0
    #
    occupancy_grid = np.zeros(
        map_img.shape,
        dtype=np.uint8
    )

    occupancy_grid[map_img < 100] = 1

    occupancy_grid[map_img >= 100] = 0

    print("\n===== OCCUPANCY GRID =====\n")

    print(
        f"Width : {occupancy_grid.shape[1]}"
    )

    print(
        f"Height: {occupancy_grid.shape[0]}"
    )

    obstacle_count = np.sum(
        occupancy_grid == 1
    )

    free_count = np.sum(
        occupancy_grid == 0
    )

    print(
        f"Obstacle cells : {obstacle_count}"
    )

    print(
        f"Free cells     : {free_count}"
    )

    print("\n==========================\n")

    #
    # Visualization
    #
    display = np.where(
        occupancy_grid == 1,
        0,
        255
    ).astype(np.uint8)

    cv2.imshow(
        "Occupancy Grid",
        display
    )

    cv2.waitKey(0)

    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()

