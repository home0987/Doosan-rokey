'''

USB 웹캠의 초점 거리를 구하려면, 
웹캠에서 실시간으로 이미지를 캡처하고
 초점 거리 계산을 수행할 수 있다

📌 실험 준비
1️⃣ 필요한 장비
✅ 카메라 (웹캠, USB 카메라, 스마트폰 카메라 등)
✅ 기준 물체 (크기가 알려진 물체, 예: A4 용지 - 21cm × 29.7cm)
✅ 측정 도구 (줄자 또는 거리 측정 앱)

📌 실험 방법
📍 1. 기준 물체(A4 용지)를 벽이나 책상에 고정
A4 용지의 너비(21cm) 를 알고 있으므로 기준 물체로 사용 가능
같은 크기의 박스, 책, 카드도 사용 가능
📍 2. 카메라를 고정하고 일정 거리 유지
카메라와 물체 사이의 거리(예: 50cm) 를 줄자로 측정
정확한 거리를 유지하는 것이 중요
📍 3. 카메라로 기준 물체 촬영
카메라를 정면에서 기준 물체를 촬영
촬영한 이미지를 test_image.jpg 로 저장
📍 4. 측정 코드 실행
촬영한 이미지를 코드에 넣고 실행
초점 거리(f) 계산 결과를 확인


📌 실험 결과 예시
✅ 입력값:

기준 물체: A4 용지 (너비 21cm)
카메라-물체 거리: 50cm
이미지 속 A4 용지의 너비(픽셀 단위): 400px
✅ 출력값 (계산된 초점 거리 f):

shell
복사
편집
Estimated Focal Length: 952.38 pixels



# -----------------------------
초점 거리:
카메라에서, 이미지 센서까지 빛이 모이는 거리를 의미

focal length = (object width in pixels) * (known distance) / (known width)
'''

import cv2
import numpy as np

def measure_focal_length(known_distance, known_width, image):
    """
    삼각 측량을 이용하여 카메라의 초점 거리(f) 측정
    :param known_distance: 실제 물체와 카메라 사이의 거리 (cm)
    :param known_width: 실제 물체의 너비 (cm)
    :param image: 측정할 이미지 (OpenCV 이미지 객체)
    :return: 초점 거리 f (픽셀 단위)
    """
    # 그레이스케일 변환 및 엣지 검출
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    
    # 윤곽선 검출
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        raise ValueError("윤곽선을 찾을 수 없습니다.")
    
    # 가장 큰 윤곽선 찾기
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    # 초점 거리 계산 (f = (픽셀 크기 * 실제 거리) / 실제 크기)
    focal_length = (w * known_distance) / known_width
    
    return focal_length

def capture_image_from_webcam():
    # 웹캠 열기 (기본 웹캠: 0번 포트)
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        raise ValueError("웹캠을 열 수 없습니다.")
    
    print("웹캠이 켜졌습니다. 캡처하려면 's'를 눌러주세요.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        
        # 실시간 화면에 프레임 표시
        cv2.imshow("Webcam", frame)
        
        # 's' 키를 눌러서 캡처
        # 카메라 로드 창에서 s키를 눌러 캡쳐
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # 's' 키를 누르면 촬영 후 저장
            print("이미지를 캡처하고 저장합니다...")
            cv2.imwrite("capture_image.jpg", frame)
            break
    
    cap.release()
    cv2.destroyAllWindows()

# 테스트용 코드
if __name__ == "__main__":
    # 실험 
    KNOWN_DISTANCE = 53  # cm (카메라와 물체 사이의 거리_ 책상 높이에서 시행)
    KNOWN_WIDTH = 21.0  # cm (A4 용지 너비)

    # 웹캠에서 이미지 캡처
    capture_image_from_webcam()

    # 캡처한 이미지 로드
    image = cv2.imread("capture_image.jpg")
    
    if image is None:
        raise ValueError("캡처한 이미지를 로드할 수 없습니다.")
    
    # 초점 거리 측정
    focal_length = measure_focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, image)
    print(f"Estimated Focal Length: {focal_length:.2f} pixels")

'''
웹캠이 켜졌습니다. 캡처하려면 's'를 눌러주세요.
qt.qpa.plugin: Could not find the Qt platform plugin "wayland" in "/home/kante/anaconda3/envs/myenv/lib/python3.9/site-packages/cv2/qt/plugins"
이미지를 캡처하고 저장합니다...
Estimated Focal Length: 75.71 pixels => 75로 사용하겠음,
'''