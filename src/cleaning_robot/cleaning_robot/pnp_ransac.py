import cv2
import numpy as np

# === (1) 기준 이미지 로드 및 ORB 특징점 검출 ===
ref_img = cv2.imread("man_orig.png", cv2.IMREAD_GRAYSCALE)  # 기준 사진 로드
orb = cv2.ORB_create()

# 기준 사진에서 특징점 검출
ref_keypoints, ref_descriptors = orb.detectAndCompute(ref_img, None)

# 기준 이미지 특징점 표시
ref_img_display = cv2.drawKeypoints(ref_img, ref_keypoints, None, color=(0, 255, 0))
cv2.imshow("Reference Image Keypoints", ref_img_display)

# === (2) 웹캠 실행 ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

# 카메라 내부 파라미터
cameraMatrix = np.array([
    [200.998, 0.0, 125.136],  
    [0.0, 200.998, 130.544],  
    [0.0, 0.0, 1.0]           
], dtype=np.float32)

# 왜곡 계수
dist_coeffs = np.array([
    13.3801, -150.4605, 0.0022977, 0.0011334, 541.2130, 
    13.1323, -148.4299, 533.8889
], dtype=np.float32)

# === (3) 웹캠에서 특징점 검출 및 PnP 적용 ===
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # ORB 디스크립터 매칭

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    if descriptors is not None and ref_descriptors is not None:
        matches = bf.match(ref_descriptors, descriptors)
        matches = sorted(matches, key=lambda x: x.distance)  # 거리순 정렬

        if len(matches) >= 4:
            # 상위 4개 매칭된 특징점 선택
            ref_matched_pts = np.array([ref_keypoints[m.queryIdx].pt for m in matches[:4]], dtype="double")
            cam_matched_pts = np.array([keypoints[m.trainIdx].pt for m in matches[:4]], dtype="double")

            # 3D 좌표로 변환 (가정: 사진의 특징점을 월드 좌표에서 특정 위치에 배치)
            points_3D = np.array([
                (p[0] * 0.1, p[1] * 0.1, 0.0) for p in ref_matched_pts  # 2D → 3D 변환
            ], dtype="double")

            # PnP 계산
            retval, rvec, tvec, inliers = cv2.solvePnPRansac(
                points_3D, cam_matched_pts, cameraMatrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )

            if retval:
                print("\nTranslation Vector (t):\n", tvec)

                # 특징점 그리기
                frame = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)

    cv2.imshow("ORB Feature Detection & PnP", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC 키를 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()