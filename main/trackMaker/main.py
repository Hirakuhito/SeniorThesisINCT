import numpy as np
import pybullet as p
import pybullet_data as pd

import vertex_data_generator as vdg


def main():
    print("\n~ Welcome ~\n")

    start_pos = [0, 0]
    end_pos = [10, 0]

    filename = "test_data"
    center_line = vdg.gen_straight_track(start_pos, end_pos, resolution=10, show=False)
    normal, vertices, mesh = vdg.gen_mesh_data(center_line, show=False)
    vdg.save_obj(filename, normal, vertices, mesh)


if __name__ == "__main__":
    main()