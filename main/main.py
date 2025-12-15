import time

import numpy as np
import pybullet as p
import pybullet_data as pd
import trackMaker.track_info_generator as pg


def main():
    print("# Welcome")

    points = pg.gen_center_point(10, 50)
    mesh_points, tangent_vector = pg.gen_mesh_data(points, 10, 50)
    pg.export_obj(mesh_points, "test_circuit")

    
if __name__ == "__main__":
    main()