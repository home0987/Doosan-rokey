import rclpy
import cv2
import numpy as np
from image_matcher import ImageMatcher

# 최외곽선 컨투어 -> 꼭짓점 좌표 반환
def find_contours_and_corners(frame, expected_size):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 그레이스케일 변환
    blurred = cv2.GaussianBlur(gray, (5, 5), 0) # 블러링 (노이즈 제거)
    edges = cv2.Canny(blurred, 50, 150) # Canny 엣지 검출
    
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # 컨투어 찾기
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True) # 컨투어 근사화
        if len(approx) == 4: # 꼭짓점이 4개인 경우만 고려
            x, y, w, h = cv2.boundingRect(approx)
            if abs(w - expected_size[0]) < 5 and abs(h - expected_size[1]) < 5:
                return approx.reshape(4, 2) # 꼭짓점 좌표 반환
    return None # 해당하는 객체가 없으면 None 반환

def main():
    rclpy.init()
    
    matcher = ImageMatcher()
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return
    
    object_sizes = {"man": (23, 18), "ext": (18, 18)}
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = matcher.orb.detectAndCompute(gray_frame, None)

        if descriptors is not None:
            for label in matcher.ref_keypoints.keys():
                ref_keypoints = matcher.ref_keypoints[label]
                ref_descriptors = matcher.ref_descriptors[label]

                if ref_descriptors is None or descriptors is None:
                    continue
                
                matches = matcher.bf.match(ref_descriptors, descriptors) # 매칭 수행
                matches = sorted(matches, key=lambda x: x.distance) # 거리 기준으로 정렬
                good_matches = matches[:50] # 상위 50개 선택
                
                match_img = cv2.drawMatches(
                    matcher.ref_images[label], ref_keypoints,
                    gray_frame, keypoints,
                    good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                cv2.imshow(f"Matching: {label}", match_img)
                
                # 컨투어를 찾아 꼭짓점 좌표 반환
                corners = find_contours_and_corners(frame, object_sizes[label])
                if corners is not None:
                    print(f"{label} corners: {corners}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
