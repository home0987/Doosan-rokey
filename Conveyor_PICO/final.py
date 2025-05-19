import time
import serial
import numpy as np
import os
import cv2
from PIL import Image
import requests
from requests.auth import HTTPBasicAuth
import sqlite3
import uuid
from datetime import datetime
import keyboard
from rembg import remove
# 시리얼 포트 설정
ser = serial.Serial("/dev/ttyACM0", 9600)
# 데이터베이스 연결
db_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../2. gradio-demos/pico.db')
# 데이터베이스 연결
conn = sqlite3.connect(db_path)
cursor = conn.cursor()
query = '''
SELECT * FROM 피코
'''
cursor.execute(query)
results = cursor.fetchall()
# # 결과 출력
for row in results:
    print(row)

# 데이터 삽입 함수
def insert_data(datetime_value, uuid_value, is_defective, image_url, defect_reason=None):
    insert_query = '''
    INSERT INTO 피코 (datetime, uuid, is_defective, image_url, defect_reason)
    VALUES (?, ?, ?, ?, ?)
    '''
    cursor.execute(insert_query, (datetime_value, uuid_value, is_defective, image_url, defect_reason))
    conn.commit()
# superbModel 함수
def superbModel(path):
    # 이미지 경로를 받아서 분석 요청
    image_path = path
    image = Image.open(image_path)
    image = np.array(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    _, img_encoded = cv2.imencode(".jpg", image)
    ACCESS_KEY = 'RM6dU9G9K05me2jsNSLXh3HMAFEoNLMH1C6rsY6W'
    response = requests.post(
        url="https://suite-endpoint-api-apne2.superb-ai.com/endpoints/a3c73094-5e81-4302-9a99-e91068c3bec1/inference",
        auth=HTTPBasicAuth("kdt2025_1-21", ACCESS_KEY),
        headers={"Content-Type": "image/jpeg"},
        data=img_encoded.tobytes(),
    )
    response_data = response.json()
    
    return image, response_data

def remove_background_with_rembg(image):
    pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    output = remove(pil_image)
    result_image = np.array(output)
    return cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)

def boundingBox(image, response_data, img_filename):
    print(response_data['objects'])
    for obj in response_data['objects']:
        # 박스 좌표 가져오기
        x1, y1, x2, y2 = obj['box']
        start_point = (x1, y1)
        end_point = (x2, y2)
        # 박스 그리기
        color = (0, 255, 0)  # 초록색
        thickness = 2  # 두께
        cv2.rectangle(image, start_point, end_point, color, thickness)
        # 텍스트 추가
        text = f"{obj['class']} ({obj['score']:.2f})"
        text_position = (x1, y1 - 10)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        text_thickness = 2
        # cv2.putText(image, text, text_position, font, font_scale, color, text_thickness, cv2.LINE_AA)
    # BGR 이미지를 RGB로 변환
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    cv2.imwrite(img_filename, image)
    
# 카메라 초기화
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("Camera Error")
    exit(-1)
# 저장할 폴더 설정
save_folder = 'processed_images'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)
# Crop 영역 설정 (x, y는 좌상단 좌표, w는 너비, h는 높이)
CROP_X, CROP_Y, CROP_W, CROP_H = 100, 100, 400, 220  # 필요에 따라 값 수정
# 경계선 검출 및 회전 함수
def process_and_rotate(img):
    # 1. 이미지 Crop
    cropped_img = img[CROP_Y:CROP_Y + CROP_H, CROP_X:CROP_X + CROP_W]
    # 2. 그레이스케일 변환 및 블러링
    gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # 블러 강도 조정 가능
    # 3. 에지 검출 (Canny)
    edges = cv2.Canny(blurred, 50, 150)
    # 에지 이미지 확인 (PIL로 보여주기)
    edges_pil = Image.fromarray(edges)
    # edges_pil.show(title="Edges")  # 디버깅용, 에지 이미지 확인
    # 4. 윤곽선 검출
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 윤곽선 그리기 (검출된 윤곽선 표시)
    img_with_contours = cropped_img.copy()
    cv2.drawContours(img_with_contours, contours, -1, (0, 0, 255), 2)  # 윤곽선 색상은 빨간색 (0,0,255)
    # 윤곽선이 없는 경우 Crop한 이미지 반환
    if not contours:
        return cropped_img, img_with_contours
    # 5. 윤곽선 근사화
    longest_contour = None
    max_length = 0
    for contour in contours:
        # 근사화
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        # 근사화된 윤곽선 길이
        length = cv2.arcLength(approx, True)
        if length > max_length:
            max_length = length
            longest_contour = approx
    # 윤곽선이 발견되지 않은 경우
    if longest_contour is None:
        return cropped_img, img_with_contours
    # 6. Hough 변환을 사용하여 직선 검출
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
    if lines is not None:
        max_line_length = 0
        best_line = None
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if length > max_line_length:
                max_line_length = length
                best_line = (x1, y1, x2, y2)
        if best_line is not None:
            x1, y1, x2, y2 = best_line
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        else:
            angle = 0
    else:
        angle = 0
    # 7. 최종 회전 각도 조정 (긴 변이 항상 수평)
    if angle < -45:
        angle += 90
    # 8. 원본 이미지에서 회전 각도만큼 회전
    (h, w) = img.shape[:2]
    center = (w // 2, h // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    # 회전 후 이미지 크기를 원본 크기에 맞추고, 회전 각도만큼만 회전하도록 설정
    rotated = cv2.warpAffine(img, rotation_matrix, (w, h))
    # 회전된 이미지 PIL로 변환하여 표시
    rotated_pil = Image.fromarray(cv2.cvtColor(rotated, cv2.COLOR_BGR2RGB))
    # rotated_pil.show(title="Rotated Image")
    return rotated, img_with_contours
# 실시간 처리 루프
ret, img = camera.read()
while True:
    # 시리얼 데이터 읽기
    data = ser.read()
    if data == b"0":  # 신호가 0일 때
        time.sleep(2)
        get_img()_
        if not ret:
            print("Failed to capture image")
            continue
        # 이미지 처리 (경계선 검출 및 회전)
        # rotated_img, img_with_contours = process_and_rotate(img)
        # # 회전된 이미지와 윤곽선 이미지 화면에 출력
        # rotated_pil_img = Image.fromarray(cv2.cvtColor(rotated_img, cv2.COLOR_BGR2RGB))  # BGR -> RGB 변환
        # contours_pil_img = Image.fromarray(cv2.cvtColor(img_with_contours, cv2.COLOR_BGR2RGB))  # BGR -> RGB 변환
        #rotated_pil_img.show(title="Rotated Image")
        # contours_pil_img.show(title="Contours Image")
        # 현재 시간으로 파일명 생성
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        img_filename = os.path.join(save_folder, f"image_{timestamp}.jpg")
        box_filename = os.path.join('../2. gradio-demos/static', f"box_image_{timestamp}.jpg")
        # 처리된 이미지 저장
        # cv2.imwrite(img_filename, rotated_img)
        cv2.imwrite(img_filename, img)
        if not os.path.exists(img_filename):
            print(f"Error: {img_filename} not found after saving")
            continue
        # superbModel API 호출
        image, dic = superbModel(img_filename)
        boundingBox(image, dic, box_filename)

        pico_dic = {'Raspberry PICO':0, 'BOOTSEL':0, 'OSCILLATOR':0, 'CHIPSET':0, 'HOLE':0, 'USB':0}
        for i in dic['objects']:
            pico_dic[i['class']] += 1
        print(pico_dic)
        print(pico_dic.values())
        if list(pico_dic.values()) < [1, 1, 1, 1, 4, 1]:
            is_defective = 1
        else:
            is_defective = 0

        datetime_value = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 현재 날짜와 시간
        uuid_value = str(uuid.uuid4())  # UUID 생성
        defect_reason = None
        image_url = f"box_image_{timestamp}.jpg"

        insert_data(datetime_value, uuid_value, is_defective, image_url, defect_reason)
        
        print(f"Image saved: {img_filename}")
        # 시리얼 신호 전송
        ser.write(b"1")

        if data != b"1":
            time.sleep(1) 
        
       
# 자원 해제
camera.release()
cv2.destroyAllWindows()
conn.close()
