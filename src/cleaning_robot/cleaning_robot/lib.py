import numpy as np
import cv2, os
from ament_index_python.packages import get_package_share_directory
pkg_path = get_package_share_directory('cleaning_robot')
img_dir_path = os.path.join(pkg_path, 'imgs')

def visual_map_topic(height:int, width:int, map_:np.int8, current_pos:tuple):
    '''/map 토픽 시각화'''
    # 가상의 10x10 Occupancy Grid 데이터 생성
    #width, height = 10, 10
    #map_ = np.random.choice([0, 100, -1], size=(height, width))

    # -1(미확인 영역)을 회색(127)으로 변환
    visual_map = np.where(map_ == -1, 127, map_)

    # 현재좌표
    visual_map[current_pos] = 100

    # 100(장애물)은 검은색(0), 0(빈 공간)은 흰색(255)으로 변환
    visual_map = np.where(visual_map == 100, 0, 255).astype(np.uint8)

    # OpenCV로 시각화
    cv2.imshow("Occupancy Grid", visual_map)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

def img_matching(img):
    people_img = os.path.join(img_dir_path, '20250225_211950.jpg')
    image_file1 = people_img

    image1 = cv2.imread(image_file1)
    image2 = img

    gray1 = cv2.cvtColor(image1,cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(image2,cv2.COLOR_BGR2GRAY)
    # ORB Detector
    sift = cv2.SIFT_create()

    # Find keypoints and descriptors
    keypoints1, descriptors1 = sift.detectAndCompute(gray1, None)
    keypoints2, descriptors2 = sift.detectAndCompute(gray2, None)

    # Create BFMatcher and match descriptors
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)

    if len(matches) > 50 :

        # Draw the matches
        match_visualization = cv2.drawMatches(
            gray1, keypoints1, gray2, keypoints2, matches[:30], None, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS
        )

        # Convert to RGB for matplotlib and plot
        # Note - opencv uses BGR image. matplotlib uses RGB image
        match_visualization_rgb = cv2.cvtColor(match_visualization, cv2.COLOR_BGR2RGB)
        match_visualization_rgb = cv2.resize(match_visualization_rgb, (640,480))
        cv2.imshow("match imgs", match_visualization_rgb)
        cv2.waitKey(1)  # OpenCV 윈도우 업데이트를 위한 키 입력 대기
        cv2.destroyWindow('math imgs')


if __name__ == '__main__':
    print('?')