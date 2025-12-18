import sys
import time

import car_functions as cf
import numpy as np
import pybullet as p
import pybullet_data as pd
import trackMaker.track_info_generator as pg


def main(engine_mode):
    print("# Welcome")

    #*================= export circuit ==================
    circuit_name = "circuit"
    runoff_name = "runoff"

    straight = 7
    radius = 3
    width = 2

    points = pg.gen_center_point(straight, radius)

    circuit_mesh_points, cricuite_tangent_vector = pg.gen_mesh_data(points, width, radius, in_out="in")
    pg.export_obj(circuit_mesh_points, circuit_name, in_out="in")
    
    runoff_mesh_points, cricuite_tangent_vector = pg.gen_mesh_data(points, width, radius, in_out="out")
    pg.export_obj(runoff_mesh_points, runoff_name, in_out="out")

    #*=========== PuBullet basic settings ===============
    match engine_mode:
        case "gui":
            engine = p.connect(p.GUI)
        case "direct":
            engine = p.connect(p.DIRECT)
        case _:
            engine = p.connect(p.GUI)

    p.setAdditionalSearchPath(pd.getDataPath())
    p.resetSimulation()
    p.setTimeStep(1. / 240.)
    p.setGravity(0, 0, -9.81)

    #*===================== load models ===================
    #* load circuit
    circuit_file_path = "trackData/" + circuit_name + ".obj"
    runoff_file_path = "trackData/" + runoff_name + ".obj"

    base_pos = [0, 0, 0]
    base_orient = p.getQuaternionFromEuler([0, 0, 0])

    circuit_vis_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=circuit_file_path,
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[0.5, 0.5, 0.5, 1],
        specularColor=[0, 0, 0]
    )

    circuit_coll_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=circuit_file_path,
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        meshScale=[1.0, 1.0, 1.0]
    )
    
    circuit_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=circuit_coll_id,
        baseVisualShapeIndex=circuit_vis_id,
        basePosition=base_pos,
        baseOrientation=base_orient
    )

    runoff_vis_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=runoff_file_path,
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[0.2, 0.45, 0.2, 1],
        specularColor=[0, 0, 0]
    )

    runoff_coll_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=runoff_file_path,
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        meshScale=[1.0, 1.0, 1.0]
    )

    runoff_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=runoff_coll_id,
        baseVisualShapeIndex=runoff_vis_id,
        basePosition=base_pos,
        baseOrientation=base_orient
    )


    #* load car
    car_path = "./formular/formular_car/car.urdf"
    car_base_pos = [radius, 0, 0.5]
    car_base_orient = p.getQuaternionFromEuler([0, 0, 0])

    car_id = p.loadURDF(car_path, basePosition=car_base_pos, baseOrientation=car_base_orient, globalScaling=0.2)

    #* load plane
    wheel_index = [1, 3, 5, 7]
    for _, wheel in enumerate(wheel_index):
        p.changeVisualShape(car_id, wheel, specularColor=[0, 0, 0])
        p.changeDynamics(car_id, wheel, lateralFriction=1.0, rollingFriction=0.1)


    #*===================== simulate =====================
    try:
        while p.isConnected():
            p.stepSimulation()
            print(cf.isContact(car_id, circuit_id, runoff_id, wheel_index))
            time.sleep(1./240.)

    except Exception as e:
        print(e)
         # シミュレーションの終了
        p.disconnect()
        print("PyBulletシミュレーションを終了しました。")

    #*====================== Debug ========================
    

    
if __name__ == "__main__":
    arg = sys.argv

    if not isinstance(arg[1], str):
        raise ValueError("CommandLine argument must be string.")

    if arg[1] != "gui" and arg[1] != "direct":
        raise ValueError("Engin mode must choose gui or direct.")

    main(engine_mode=arg[1])