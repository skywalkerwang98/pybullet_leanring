import time
import pybullet
import pybullet_data

clent = pybullet.connect(pybullet.GUI)
# client = pybullet.connect(pybullet.DIRECT)
# pybullet.GUI：服务端打开图形GUI做渲染，需要独显，性能消耗大
# pybullet.DIRECT：不打开图形渲染，性能消耗小

# pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
# pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
# pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)
# pybullet.COV_ENABLE_RENDERING：是否渲染
# pybullet.COV_ENABLE_GUI：是否打开控件
# pybullet.COV_ENBALE_TINY_RENDERER：是否使用核显渲染

# 添加资源路径
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置重力，一般设置z轴为g，即-9.8
pybullet.setGravity(0, 0, -9.8)

# 加载模型，场景和机器人
sceneID = pybullet.loadURDF('plane.urdf')
# 加载机器人时要给出初始的位姿，用四元数xyzw表示
robotID = pybullet.loadURDF('r2d2.urdf', [0, 0, 0], [0, 0, 0, 1])

# 迭代运行
# while(True):
#     pybullet.stepSimulation()
#     time.sleep(0.05)

# 或者实时运行
pybullet.setRealTimeSimulation(1)
while(True):
    pass

# 关闭服务器
pybullet.disconnect()
