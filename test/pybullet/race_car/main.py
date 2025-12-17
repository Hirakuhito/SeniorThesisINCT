import pybullet as p
import pybullet_data
import time

class ControllRacecar():
    def __init__(self,setspeed):
        self.setspeed = setspeed
    
    def up(self):
        speedlist = [self.setspeed,self.setspeed,0,0]
        return speedlist
    
    
    def back(self):
        self.setspeed *= -1
        speedlist = [self.setspeed,self.setspeed,0,0]
        self.setspeed *= -1
        return speedlist
    
    def left(self):
        speedlist = [self.setspeed*0.5,self.setspeed*1.5,self.setspeed*0.5,self.setspeed*1.5]
        return speedlist
    
    def right(self):
        speedlist = [self.setspeed*1.5,self.setspeed*0.5,self.setspeed*1.5,self.setspeed*0.5]
        return speedlist
    
    def stop(self):
        speedlist = [0,0,0,0]
        return speedlist
    
def main():
    contrallrace = ControllRacecar(10)
    # --- PyBulletの初期設定 ---
    physicsClient = p.connect(p.GUI)

    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    timestep = 1. / 240.
    p.setTimeStep(timestep)

    planeId = p.loadURDF("plane.urdf")

    StartPos = [0,0,0]
    StartOrient = p.getQuaternionFromEuler([0,0,3.14])
    file_path = "racecar/racecar.urdf"
    carId = p.loadURDF(file_path,StartPos, StartOrient)

    p.resetDebugVisualizerCamera(5, 45, -30, StartPos)

    running = True
    try:
        while running:
            keys = p.getKeyboardEvents()

            # ASCII値 'u' は 117
            # ASCII値 'd' は 100
            # ASCII値 'r' は 114
            # ASCII値 'l' は 108
            # ASCII値 'q' は 113
            
            if 117 in keys and keys[117] & p.KEY_IS_DOWN:
                p.setJointMotorControlArray(carId,[2,3,5,7],p.VELOCITY_CONTROL,targetVelocities=contrallrace.up())
            elif 100 in keys and keys[100] & p.KEY_IS_DOWN:
                p.setJointMotorControlArray(carId,[2,3,5,7],p.VELOCITY_CONTROL,targetVelocities=contrallrace.back())
            elif 114 in keys and keys[114] & p.KEY_IS_DOWN:
                p.setJointMotorControlArray(carId,[2,3,5,7],p.VELOCITY_CONTROL,targetVelocities=contrallrace.right())
            elif 108 in keys and keys[108] & p.KEY_IS_DOWN:
                p.setJointMotorControlArray(carId,[2,3,5,7],p.VELOCITY_CONTROL,targetVelocities=contrallrace.left())
            else:
                p.setJointMotorControlArray(carId,[2,3,5,7],p.VELOCITY_CONTROL,targetVelocities=contrallrace.stop())
            
            p.stepSimulation()
            
            if 113 in keys and keys[113] & p.KEY_IS_DOWN:
                print("シミュレーションを終了します。")
                running = False # ループを終了させるフラグをFalseに設定
                
            time.sleep(timestep)

    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        p.disconnect()
        print("シミュレーションが終了しました。")
        
        
if __name__ == "__main__":
    main()