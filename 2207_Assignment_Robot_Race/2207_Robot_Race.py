
import pybullet as p
import pybullet_data as pd
import time
def starter2():
    p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) 
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(plane_shape, plane_shape)
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(1)
      
def createRobots():
    rob1 = p.loadURDF('robot0_fom_discussion.py')
    p.resetBasePositionAndOrientation(rob1, [2, 0, 0.7], [0, 0, 0, 1])

    time.sleep(2)
    rob2 = p.loadURDF('102_xyz.urdf')
    p.resetBasePositionAndOrientation(rob2, [0, 2, 0.6], [0, -2, 0, 1])
    time.sleep(2)

    rob3 = p.loadURDF('102_box.urdf')
    p.resetBasePositionAndOrientation(rob3, [0, -2, 1], [0, 0, 0, 1])


    return [rob1, rob2, rob3]     
def runRobots(list_robots):
    for index_robot in list_robots:
        for index_joint in range(p.getNumJoints(index_robot)):
            speed = 1
            p.setJointMotorControl2(index_robot, # obj ID
                                index_joint, # joint ID
                                controlMode =  p.VELOCITY_CONTROL,
                                targetVelocity = speed)

def main():
    starter2()
    list_robots = createRobots()
    runRobots(list_robots)

if __name__ == "__main__":
	main()
      