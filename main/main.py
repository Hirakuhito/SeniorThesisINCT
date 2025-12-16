import time

import numpy as np
import pybullet as p
import pybullet_data as pd
import trackMaker.track_info_generator as pg


def main():
    print("# Welcome")

    engine = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    #*===================== load models ===================
    #* load circuit
    circuit_file_path = "trackData/test_circuit.obj"

    circuit_vis_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=circuit_file_path,
        meshScale=[1, 1, 1]
        rgbaColor=[0.5, 0.5, 0.5, 1]
    )

    circuit_coll_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=circuit_file_path,
        meshScale=[1, 1, 1]
    )

    circuit_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=circuit_coll_id,
        baseVisualShapeIndex=circuit_vis_id,
        basePosi
    )

    #* load car




    #*====================== Debug ========================
    points = pg.gen_center_point(10, 50)
    mesh_points, tangent_vector = pg.gen_mesh_data(points, 10, 50)
    pg.export_obj(mesh_points, "test_circuit")

    
if __name__ == "__main__":
    main()