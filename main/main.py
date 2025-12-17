import time

import numpy as np
import pybullet as p
import pybullet_data as pd
import trackMaker.track_info_generator as pg


def main():
    print("# Welcome")

    #*================= export circuit ==================
    straight = 60
    radius = 30
    width = 15

    points = pg.gen_center_point(straight, radius)

    circuit_mesh_points, cricuite_tangent_vector = pg.gen_mesh_data(points, width, radius, in_out="in")
    pg.export_obj(circuit_mesh_points, "test_circuit", in_out="in")
    
    runoff_mesh_points, cricuite_tangent_vector = pg.gen_mesh_data(points, width, radius, in_out="out")
    pg.export_obj(runoff_mesh_points, "test_runoff", in_out="out")

    #*=========== PuBullet basic settings ===============
    engine = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    #*===================== load models ===================
    #* load circuit
    circuit_file_path = "trackData/test_circuit.obj"
    runoff_file_path = "trackData/test_runoff.obj"

    base_pos = [0, 0, 0]
    base_orient = p.getQuaternionFromEuler([0, 0, 0])

    circuit_vis_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=circuit_file_path,
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[0.5, 0.5, 0.5, 1]
    )

    circuit_coll_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=circuit_file_path,
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
        rgbaColor=[0.0, 0.8, 0.0, 1]
    )

    runoff_coll_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=runoff_file_path,
        meshScale=[1.0, 1.0, 1.0]
    )

    runoff_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=runoff_coll_id,
        baseVisualShapeIndex=runoff_vis_id,
        basePosition=base_pos,
        baseOrientation=base_orient
    )

    #todo ランオフをマット緑にしたい
    # p.changeVisualShape(runoff_id, 0, specularColor=[0, 0, 0])

    #* load car
    car_path = "./formular/formular_car/car.urdf"
    car_base_pos = [radius, 0, 0.5]
    car_base_orient = p.getQuaternionFromEuler([0, 0, 0])

    car_id = p.loadURDF(car_path, basePosition=car_base_pos, baseOrientation=car_base_orient)

    #* load plane
    wheel_index = [1, 3, 5, 7]
    for _, wheel in enumerate(wheel_index):
        p.changeVisualShape(car_id, wheel, specularColor=[0, 0, 0])
        p.changeDynamics(car_id, wheel, lateralFriction=1.0, rollingFriction=0.1)


    #*===================== simulate =====================
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1./240.)

    except Exception:
        pass

    # シミュレーションの終了
    p.disconnect()
    print("PyBulletシミュレーションを終了しました。")

    #*====================== Debug ========================
    

    
if __name__ == "__main__":
    main()