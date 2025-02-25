import numpy as np
import cv2

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


if __name__ == '__main__':
    print('?')