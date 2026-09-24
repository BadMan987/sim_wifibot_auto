#!/usr/bin/env python3

import yaml
import cv2
import os


def main():

    yaml_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/map.yaml"

    #  map.yaml
    with open(yaml_file, "r") as file:

        map_config = yaml.safe_load(file)

    resolution = map_config["resolution"]

    origin = map_config["origin"]

    image_name = map_config["image"]

    image_path = os.path.join(
        os.path.dirname(yaml_file),
        image_name
    )

    print("\n========== MAP INFO ==========\n")

    print("Resolution :", resolution)

    print("Origin     :", origin)

    print("Image Path :", image_path)

    #  pgm 
    map_img = cv2.imread(
        image_path,
        cv2.IMREAD_GRAYSCALE
    )

    if map_img is None:

        print("ERROR: Cannot load map image")

        return

    height, width = map_img.shape

    print("Width      :", width)

    print("Height     :", height)

    print("\n==============================\n")

    # afficher map
    cv2.imshow(
        "Occupancy Grid",
        map_img
    )

    cv2.waitKey(0)

    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()

