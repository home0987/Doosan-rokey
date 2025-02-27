import numpy as np
import cv2, os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Transform

pkg_path = get_package_share_directory('cleaning_robot')
img_dir_path = os.path.join(pkg_path, 'imgs')

def quaternion_from_matrix(matrix):
    """회전 행렬(3x3 또는 4x4)을 사원수 (x, y, z, w)로 변환"""
    M = np.array(matrix, dtype=np.float64)
    if M.shape == (4, 4):
        M = M[:3, :3]  # 상위 3x3 회전 행렬 추출

    trace = np.trace(M)
    q = np.zeros(4)

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        q[3] = 0.25 * S
        q[0] = (M[2, 1] - M[1, 2]) / S
        q[1] = (M[0, 2] - M[2, 0]) / S
        q[2] = (M[1, 0] - M[0, 1]) / S
    else:
        if M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
            S = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2
            q[3] = (M[2, 1] - M[1, 2]) / S
            q[0] = 0.25 * S
            q[1] = (M[0, 1] + M[1, 0]) / S
            q[2] = (M[0, 2] + M[2, 0]) / S
        elif M[1, 1] > M[2, 2]:
            S = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2
            q[3] = (M[0, 2] - M[2, 0]) / S
            q[0] = (M[0, 1] + M[1, 0]) / S
            q[1] = 0.25 * S
            q[2] = (M[1, 2] + M[2, 1]) / S
        else:
            S = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2
            q[3] = (M[1, 0] - M[0, 1]) / S
            q[0] = (M[0, 2] + M[2, 0]) / S
            q[1] = (M[1, 2] + M[2, 1]) / S
            q[2] = 0.25 * S

    return q  # (x, y, z, w)


def pnp(points_3D, points_2D):
    # 카메라 내부 파라미터 행렬 (Intrinsic Matrix)
    cameraMatrix = np.array([
        [200.998, 0.0, 125.136],  # fx,  0, cx
        [0.0, 200.998, 130.544],  # 0,  fy, cy
        [0.0, 0.0, 1.0]           # 0,   0,  1
    ], dtype=np.float32)
    #cameraMatrix = 0.001 * cameraMatrix
    # 왜곡 계수
    dist_coeffs = np.array([
        13.3801, -150.4605, 0.0022977, 0.0011334, 541.2130,
        13.1323, -148.4299, 533.8889
    ], dtype=np.float32)

    # PnP 해법 적용
    retval, rvec, tvec, _ = cv2.solvePnPRansac(
        points_3D, points_2D, cameraMatrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )

    # 회전 벡터를 회전 행렬로 변환
    R, _ = cv2.Rodrigues(rvec)

    # 결과 출력
    print("Rotation Matrix (R):\n", R)
    print("\nTranslation Vector (t):\n", tvec)
    return R, tvec

def tf_cam_2_base(x, y, z):
    # 카메라 좌표에서의 객체 위치 (예제: 카메라에서 30cm 앞, 20cm 오른쪽, 1m 위에 있음)
    #tvec_camera = np.array([[0.3], [0.2], [1.0]])
    tvec_camera = np.array([x, y, z]).transpose()

    # R_bc (회전 행렬)
    R_bc = np.array([
        [0.000,  0.000,  1.000],
        [-1.000, 0.000,  0.000],
        [0.000, -1.000,  0.000]
    ])

    # t_bc (이동 벡터)
    t_bc = np.array([
        [-0.060],
        [0.000],
        [0.244]
    ]).transpose()

    # 변환 수행 (로봇 좌표계로 변환)
    tvec_base = R_bc@tvec_camera + t_bc

     
    # 결과 출력
    print("카메라 좌표계에서의 객체 위치 (tvec_camera):\n", tvec_camera)
    print("\n로봇(Base) 좌표계에서의 객체 위치 (tvec_base):\n", tvec_base)

    return tvec_base

def gen_tf_pnp(R, tvec):
    T = np.hstack((R, tvec))  # 변환 행렬 [R | t]
    T = np.vstack((T, [0, 0, 0, 1]))  # 4x4 변환 행렬로 확장

    # 사용자 정의 quaternion 변환 함수 사용
    q = quaternion_from_matrix(T)

    # Transform 메시지 생성
    transform_msg = Transform()
    transform_msg.translation.x = float(tvec[0])
    transform_msg.translation.y = float(tvec[1])
    transform_msg.translation.z = float(tvec[2])
    transform_msg.rotation.x = float(q[0])
    transform_msg.rotation.y = float(q[1])
    transform_msg.rotation.z = float(q[2])
    transform_msg.rotation.w = float(q[3])

    return transform_msg

def quaternion_to_rotation_matrix(x, y, z, w):
    """사원수를 3x3 회전 행렬로 변환"""
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return R

def transform_to_matrix(transform_msg):
    """ROS Transform 메시지를 4x4 변환 행렬로 변환"""
    tx, ty, tz = transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z
    qx, qy, qz, qw = transform_msg.rotation.x, transform_msg.rotation.y, transform_msg.rotation.z, transform_msg.rotation.w

    # 사원수를 회전 행렬로 변환
    R = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    
    # 4x4 변환 행렬 생성
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    
    return T

if __name__ == '__main__':
    print('?')