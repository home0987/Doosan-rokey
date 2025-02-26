import rclpy
import cv2
import numpy as np
from image_matcher import ImageMatcher

def main():
    rclpy.init()
    
    matcher = ImageMatcher()
    
    # 웹캠 열기 (기본 카메라: 0)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 프레임을 Grayscale 변환
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ORB 특징점 검출
        keypoints, descriptors = matcher.orb.detectAndCompute(gray_frame, None)

        if descriptors is not None:
            for label in matcher.ref_keypoints.keys():
                ref_keypoints = matcher.ref_keypoints[label]
                ref_descriptors = matcher.ref_descriptors[label]

                if ref_descriptors is None or descriptors is None:
                    continue  # 디스크립터가 없는 경우 패스

                # 매칭 수행
                matches = matcher.bf.match(ref_descriptors, descriptors)
                matches = sorted(matches, key=lambda x: x.distance)
                good_matches = matches[:50]  # 상위 50개 선택

                # 매칭 결과를 이미지에 그리기
                match_img = cv2.drawMatches(
                    matcher.ref_images[label], ref_keypoints,
                    gray_frame, keypoints,
                    good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                
                # # 매칭된 특징점 좌표 추출
                # matched_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches])

                # # 코너 검출 적용
                # if len(matched_pts) > 0:
                #     matched_gray = np.zeros_like(gray_frame)
                #     for pt in matched_pts:
                #         cv2.circle(matched_gray, (int(pt[0]), int(pt[1])), 3, 255, -1)

                #     corners = cv2.goodFeaturesToTrack(matched_gray, maxCorners=10, qualityLevel=0.3, minDistance=10)
                    
                #     if corners is not None:
                #         corners = np.int0(corners)  # 정수 변환
                #         for corner in corners:
                #             x, y = corner.ravel()
                #             cv2.circle(match_img, (x, y), 5, (0, 255, 0), -1)  # 코너 표시

                cv2.imshow(f"Matching: {label}", match_img)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
