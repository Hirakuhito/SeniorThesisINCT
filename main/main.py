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
    track_name = "track"
    runoff_name = "runoff"

    straight = 7
    radius = 3
    width = 2

    points = pg.gen_center_point(straight, radius)

    track_mesh_points, track_tangent_vector = pg.gen_mesh_data(points, width, radius, in_out="in")
    pg.export_obj(track_mesh_points, track_name, in_out="in")
    
    runoff_mesh_points, _ = pg.gen_mesh_data(points, width, radius, in_out="out")
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
    track_file_path = "circuitData/" + track_name + ".obj"
    runoff_file_path = "circuitData/" + runoff_name + ".obj"

    base_pos = [0, 0, 0]
    base_orient = p.getQuaternionFromEuler([0, 0, 0])

    track_vis_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=track_file_path,
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[0.5, 0.5, 0.5, 1],
        specularColor=[0, 0, 0]
    )

    track_coll_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=track_file_path,
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        meshScale=[1.0, 1.0, 1.0]
    )
    
    track_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=track_coll_id,
        baseVisualShapeIndex=track_vis_id,
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
    car_base_pos = [radius, 0, 0.2]
    car_base_orient = p.getQuaternionFromEuler([0, 0, 0])

    car_id = p.loadURDF(car_path, basePosition=car_base_pos, baseOrientation=car_base_orient, globalScaling=0.2)

    #* load plane
    wheel_index = [1, 3, 5, 7]
    for _, wheel in enumerate(wheel_index):
        p.changeVisualShape(car_id, wheel, specularColor=[0, 0, 0])
        p.changeDynamics(car_id, wheel, lateralFriction=1.0, rollingFriction=0.1)

    #* Dictionary of models
    name_to_id = {"car":car_id, "track":track_id, "runoff":runoff_id}
    id_to_name = {car_id:"car", track_id:"track", runoff_id:"runoff"}

    #*===================== simulate =====================
    try:
        while p.isConnected():
            p.stepSimulation()
            hit_info = cf.checkHit(car_id, f_or_b='f', show=False, dict=id_to_name)
            # print(f"Hit Info : Is Hit -> {hit_info[0]}, Hit to {hit_info[1]}, Distance {hit_info[2]}")
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