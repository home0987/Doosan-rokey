import cv2
import time
import os
import globals

save_folder = 'dataset'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

camera = cv2.VideoCapture(0)
video = cv2.VideoWriter("video.avi", -1, 25, (640, 480))
while True:
    f, img = camera.read()
    print(globals.a)
    if globals.a:
        timestamp = time.strftime("%Y%m%d_%H%M%S")  # 현재 시간을 기반으로 파일명 생성
        img_filename = os.path.join(save_folder, f"image_{timestamp}.jpg")
        print(img_filename)
        cv2.imwrite(img_filename, img)
        globals.a = False
    
    
    video.write(img)

    cv2.imshow("webcam", img)
    if cv2.waitKey(5) != -1:
        break
video.release()


# import cv2

# camera = cv2.VideoCapture(0)
# video = cv2.VideoWriter("video.avi", -1, 25, (640, 480))
# while True:
#     f, img = camera.read()

#     video.write(img)

#     cv2.imshow("webcam", img)
#     if cv2.waitKey(5) != -1:
#         break
# video.release()