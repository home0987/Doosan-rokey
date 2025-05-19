#!/usr/bin/env python3

# 5개의 waypoint -> 중앙 위치, 각 벽의 위치

class coordinate:
    def __init__(self):
        self.coordinates = {
            # 0: [-0.5, 4.0, 0.5796275077079649],
            # 1: [-5.0, 5.0, 0.1],
            # 2: [-5.0, -1.0, 0.4],
            # 3: [3.1, -0.9, 0.6],
            # 4: [0.0, 0.0, 0.2],
            0: [0.5, 0.0, 0.6],
            1: [7.2, 7.1, 0.6],
            2: [7.3, 1.3, 0.2],
            3: [4.0, 0.8, 0.2],
            4: [4.0, 3.5, 0.6],
        }

    def get_coordinate(self):
        return self.coordinates.values()

def main(args=None):
    coord = coordinate()
    coord.get_coordinate()

if __name__ == "__main__":
    main()

# 초기 init
#   stamp:
#     sec: 520
#     nanosec: 883000000
#   frame_id: map
# pose:
#   pose:
#     position:
#       x: 0.4339665907015518
#       y: 3.1949619196485486
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.8148815572267255
#       w: 0.5796275077079649