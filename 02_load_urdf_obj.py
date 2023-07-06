import time
import pybullet
import pybullet_data

if __name__ == '__main__':
    # 创建仿真环境
    client = pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    pybullet.setGravity(0, 0, -9.8)
    pybullet.setRealTimeSimulation(1)

    # 加载模型,urdf模型包含几何结构、动力学模型和碰撞属性，常用于场景和机器人模型
    # 加载场景
    pybullet.loadURDF('plane100.urdf', useMaximalCoordinates=True)
    # 载入urdf格式的机器人
    r_ind = pybullet.loadURDF('r2d2.urdf', (3, 1, 1), pybullet.getQuaternionFromEuler([0, 0, 0.785]))
    # obj只包含几何结构，一般用作场景物体，由于obj模型不带碰撞属性，因此pybullet载入时需要为obj格式文件同时加载外观，并赋予碰撞属性
    	# 视觉属性
    visual_ind = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=[0, 0, 0],
        meshScale=[1, 1, 1])

	# 碰撞属性
    collision_ind = pybullet.createCollisionShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        collisionFramePosition=[0, 0, 0],
        meshScale=[1, 1, 1])
	
	# 从视觉和碰撞属性中创建模型
    pybullet.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_ind,
        baseVisualShapeIndex=visual_ind,
        basePosition=[0, 0, 1],
        useMaximalCoordinates=True)
    
    # 运行
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
    while(True):
        time.sleep(1. / 240.)
