# 2. 카메라 이용

'''
객체가 인식되었을 때, 해당 객체의 최대(주요) 색상을 가져와서 트래킹 성능을 높이는 기능을 추가
이 과정에서 객체의 색상을 분석하여 특정 색상이 포함된 경우 가중치를 높이는 방식을 사용할 수 있을것,

YOLO로 객체 감지
객체의 Bounding Box 좌표를 가져옴
해당 영역의 주요 색상(최대 비율 색상) 분석
특정 색상(예: 빨간색)이 많이 포함되면 트래킹 가중치 증가

✅ 객체의 주요 색상을 추출하여 트래킹 가중치를 조절하는 기능 추가
✅ 특정 색상(예: 빨간색)이 많으면 트래킹 가중치를 증가하여 더욱 강력하게 추적
✅ 카메라(웹캠) 기반 실시간 객체 추적 가능


results에서 감지된 객체의 클래스는 boxes.cls에 저장되어 있으며,
 model.names를 이용해 클래스 번호에 대응하는 클래스명을 확인할 수 있

 

 
amr에게 어떤 정보를 넘길건지,,/
특정 바운딩 박스 만 넘겨줘야 한다 -> 초록색으로 바운딩 박스한, 대상에만..
'''

# camera_detection.py

import cv2
import numpy as np
from ultralytics import YOLO

def get_dominant_color(image, k=3):
    """ 주어진 이미지에서 주요 색상을 찾는 함수 """
    if image is None or image.size == 0:
        return (255, 255, 255)  # 분석 불가한 경우 흰색 반환

    pixels = np.float32(image.reshape(-1, 3))
    
    # 픽셀 수가 너무 적으면 기본 색상 반환
    if len(pixels) < k:
        return tuple(map(int, pixels.mean(axis=0)))

    _, labels, palette = cv2.kmeans(
        pixels, k, None,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0), 
        10, cv2.KMEANS_RANDOM_CENTERS
    )
    
    _, counts = np.unique(labels, return_counts=True)
    dominant_color = palette[np.argmax(counts)]
    return tuple(map(int, dominant_color))

def is_target_color(color, target='red'):
    """ 특정 색상이 목표 색상인지 판단하는 함수 """
    b, g, r = color  # OpenCV는 BGR 형식

    if target == 'red':
        return r > 100 and r > g and r > b  # 빨간색이 두드러지면 True
    elif target == 'blue':
        return b > 100 and b > g and b > r
    elif target == 'green':
        return g > 100 and g > r and g > b
    return False

# YOLO 모델 불러오기
# model = YOLO('/home/kante/spp_model/runs/detect/train/weights/best.pt')
model = YOLO('/home/kante/shopping_pat_ws/runs/detect/amr_train/weights/best.pt')

cap = cv2.VideoCapture(4)  # 웹캠 사용

# AMR 경로를 추적할 리스트
amr_path = []

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 640))  # 실시간 트래킹 시 해상도 조정

    # 객체 탐지 수행
    results = model.track(frame, persist=True)  # 트래킹 유지
    labels = results[0].boxes.cls  # 첫 번째 객체의 클래스 번호
    confidences = results[0].boxes.conf  # 신뢰도 값
    class_names = model.names  # 모델에서 학습된 클래스명들

    if results and len(results) > 0 and hasattr(results[0], 'boxes'):
        boxes = results[0].boxes  # 감지된 객체들의 Bounding Box
        for box, label, confidence in zip(boxes, labels, confidences):
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 좌표 변환

            # 클래스가 amr인 대상에만 경로를 기록하게 끔 추가함.
            # 클래스가 AMR인 경우에만 경로 기록
            if int(label) == 0:  # AMR 클래스
                # AMR의 Bounding Box 중심 계산
                x_center, y_center = (x1 + x2) // 2, (y1 + y2) // 2

                # 경로 리스트에 중심 좌표 추가
                amr_path.append((x_center, y_center))
                # 경로 시각화 (연결된 선)
                # for i in range(1, len(amr_path)):
                #     cv2.line(frame, amr_path[i - 1], amr_path[i], (0, 255, 0), 2)  # 녹색 선으로 경로 표시

                # AMR (사람 )의 중심에 원을 그려서 위치 표시
                cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)

            # 객체 영역 잘라서 색상 분석
            object_region = frame[y1:y2, x1:x2]
            dominant_color = get_dominant_color(object_region)

            # 트래킹 가중치 적용 (특정 색상이 많으면 더 강하게 추적)
            if is_target_color(dominant_color, target='red'):
                # true 이면 -> 
                class_name = class_names[int(label)]  # 클래스명
                cv2.putText(frame, f"{class_name} 0 ({confidence:.2f})", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 초록색 박스에 클래스 0과 신뢰도
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)  # 녹색 박스
            else:
                class_name = class_names[int(label)]  # 클래스명
                cv2.putText(frame, f"{class_name} 1 ({confidence:.2f})", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (192, 192, 192), 2)  # 회색 박스에 클래스 1과 신뢰도
                cv2.rectangle(frame, (x1, y1), (x2, y2), (192, 192, 192), 3)  # 회색 박스

    # 결과 시각화
    cv2.imshow("Tracking", frame)
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
