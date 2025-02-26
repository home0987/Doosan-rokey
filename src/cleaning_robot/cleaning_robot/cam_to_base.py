import numpy as np

# 카메라 좌표에서의 객체 위치 (예제: 카메라에서 30cm 앞, 20cm 오른쪽, 1m 위에 있음)
tvec_camera = np.array([[0.3], [0.2], [1.0]])

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
])

# 변환 수행 (로봇 좌표계로 변환)
tvec_base = np.dot(R_bc, tvec_camera) + t_bc

# 결과 출력
print("카메라 좌표계에서의 객체 위치 (tvec_camera):\n", tvec_camera)
print("\n로봇(Base) 좌표계에서의 객체 위치 (tvec_base):\n", tvec_base)
