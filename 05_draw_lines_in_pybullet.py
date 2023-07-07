import pybullet
import pybullet_data
from loguru import logger
import time
import numpy as np

def set_camera(robot_id: int, width: int = 224, height: int = 224, server_id: int = 0):
    # 获取当前机器人位姿
    position, orientation = pybullet.getBasePositionAndOrientation(robot_id, physicsClientId=server_id)
    # 镜头实时跟随机器人
    # 使用resetresetDebugVisualizerCamera()这个api可以设置当前镜头的视角。因为我们要让镜头跟随机器人，因此就实时获得机器人的位置，然后把镜头对准机器人：
    # pybullet.resetDebugVisualizerCamera(cameraDistance=5,
    #                                 cameraYaw=120,
    #                                 cameraPitch=-45,
    #                                 cameraTargetPosition=position)
    # resetresetDebugVisualizerCamera()的参数中，cameraTargetPosition表示镜头对准的位置，其它参数表示镜头与该位置的距离、视角方向。

    # 在机器人上设置图像传感器
    # pybullet中图像传感器的设置是一个伪相机的放置，即在机器人的位置上实时获取其视角上的图像，因此要自己设置传感器在世界坐标中的位置，以及传感器的投影矩阵。
    # 获取伪相机位姿
    # 通过computeViewMatrix()这个api设置伪相机的视角位姿，需要提供的参数为相机位置，视角位置和相机纵轴。
    # 相机位置决定成像位置，相机位置与视角位置形成的矢量方向决定了相机朝向，相机纵轴朝上并与相机朝向垂直。
    r_mat = pybullet.getMatrixFromQuaternion(orientation)
    tx_vec = np.array([r_mat[0], r_mat[3], r_mat[6]])
    ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])
    tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])

    
    camera_position = np.array(position)
    # 相对y轴有个偏移
    target_position = camera_position + 1 * ty_vec

    view_mat = pybullet.computeViewMatrix(cameraEyePosition=camera_position,
                                        cameraTargetPosition=target_position,
                                        cameraUpVector=tz_vec)
    # 获取伪相机的投影参数
    # 通过computeProjectionMatrixFOV()这个函数来设置伪相机的FOV，视距：
    proj_mat = pybullet.computeProjectionMatrixFOV(fov=60.0, # field of view
                                               aspect=1.0, # scale, default=1
                                               nearVal=0.01,  # view distance min
                                               farVal=10)  # view distance max)
    # 从相机位姿与投影参数获得图像
    w, h, rgb, depth, seg = pybullet.getCameraImage(width=width,
                                                height=height,
                                                viewMatrix=view_mat,
                                                projectionMatrix=proj_mat,
                                                physicsClientId=server_id)
    return w, h, rgb, depth, seg

def draw_pose_in_pybullet(*pose):
    """
    *Draw pose frame in pybullet based on current pose*
    :param pose: np.ndarray, shape=[4, 4] or tuple of (position, orientation)
    """
    # pybullet提供了在仿真环境中添加点线文本的api，比如addUserDebugLine, addUserDebugPoints等，并返回这些点线的id，可用于后续的删除修改。
    if len(pose) == 1:
        position = pose[0][:3, 3]
        rotation = pose[0][:3, :3]
    else:
        position, orientation = pose
        print(orientation)
        rotation = np.array(pybullet.getMatrixFromQuaternion(orientation)).reshape([3, 3])
        print(rotation)
    start_point = position
    end_point_x = position + rotation[:, 0] * 2
    end_point_y = position + rotation[:, 1] * 2
    end_point_z = position + rotation[:, 2] * 2
    pybullet.addUserDebugLine(start_point, end_point_x, [1, 0, 0])
    pybullet.addUserDebugLine(start_point, end_point_y, [0, 1, 0])
    pybullet.addUserDebugLine(start_point, end_point_z, [0, 0, 1])


if __name__ == '__main__':
    client = pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setPhysicsEngineParameter(numSolverIterations=10)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    pybullet.setGravity(0, 0, -9.8)
    pybullet.setRealTimeSimulation(1)

    shift = [0, 0, 0]
    scale = [1, 1, 1]

    visual_shape_id = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=[0, 0, 0],
        meshScale=scale)
    collision_shape_id = pybullet.createCollisionShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        collisionFramePosition=[0, 0, 0],
        meshScale=scale)
    pybullet.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 2],
        useMaximalCoordinates=True)

    plane_id = pybullet.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    cube_ind = pybullet.loadURDF('cube.urdf', (0, 1, 0), pybullet.getQuaternionFromEuler([0, 0, 0.785]))
    r_ind = pybullet.loadURDF('r2d2.urdf', (3, 1, 1),  pybullet.getQuaternionFromEuler([0, 0, 1.1]))
    # 创建结束，重新开启渲染
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    # 机器人依靠关节运动，所以通过向关节发出控制指令，就可以让机器人动起来
    # 首先要得到机器人关节信息
    # 得到关节数目
    num_joints = pybullet.getNumJoints(r_ind)

    # 得到每个关节的信息,用列表存储
    # 依次分别表示关节序号，关节名称，关节类型等等。

    # 当关节类型为pybullet.JOINT_FIXED时，是不能动的。因此要筛掉这种关节，剩下来的就可以控制了。

    # r2d2机器人是有轮子的，因此我们想获得这些控制轮子的关节，即wheel在关节名字内。
    joint_infos = []
    for i in range(num_joints):
        joint_info = pybullet.getJointInfo(r_ind, i)
        logger.info(joint_info)
        if joint_info[2] != pybullet.JOINT_FIXED:
            if 'wheel' in str(joint_info[1]):
                joint_infos.append(joint_info)


    # 获得关节后，开始控制关节的速度和加速度进行运动
    # setJointMotorControl2()是控制单个关节的api，也可以用setJointMotorControlArray()一次设置多个关节。其中，controlMode参数常用速度控制和位置控制，对应参数为targetVelocity和targetPosition，force参数限制了最大的力（加速度），防止加速度太大导致运动不稳定。速度、位置是通过PID控制获得的。
    MAX_FORCE = 1
    VELOCITY = -3.14
    for _, joint_info in enumerate(joint_infos):
        pybullet.setJointMotorControl2(bodyIndex=r_ind,
                                    jointIndex=joint_info[0],
                                    controlMode=pybullet.VELOCITY_CONTROL,
                                    targetVelocity=VELOCITY,
                                    force=MAX_FORCE)
    flag = True
    while(True):
        pybullet.removeAllUserDebugItems() # 把之前的线删除，否则会一直在仿真环境中出现
        # 碰撞检测，AABB法，即包围盒法，是一种快速的碰撞检测方法。它的原理是，将物体看成一个个的包围盒，如果两个包围盒相交，那么物体也相交。这种方法的优点是简单快速，缺点是不够精确，因此适合用于粗略的碰撞检测。
        # pybullet中分为两步进行碰撞检测，首先是通过getAABB()获得包围盒，然后通过computeAABB()计算包围盒。
        pmin, pmax = pybullet.getAABB(r_ind)
        # 通过getOverlappingObjects得到所有与AABB框相交的物体id,返回是一个列表，列表中的每个元素是一个元组，元组的第一个元素是物体id，第二个元素是碰撞体id。
        collide_ids = pybullet.getOverlappingObjects(pmin, pmax)
        logger.info(collide_ids)
        for collide_id in collide_ids:
            if collide_id[0] != r_ind:
                logger.info(f'detect robot collide with object id {r_ind}!')
        
        set_camera(r_ind)
        position, orientation = pybullet.getBasePositionAndOrientation(r_ind)
        draw_pose_in_pybullet(position, orientation)
        pybullet.stepSimulation()
        time.sleep(1./240.)
        if not flag:
            break
    

