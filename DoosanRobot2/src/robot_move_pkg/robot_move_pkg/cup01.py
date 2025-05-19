import rclpy
import math, time

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from robot_move_pkg.lib import setvel, setvelj, wait_digital_input, grab_signal, open_signal, close_signal, joint_open, joint_close, joint_grab
    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            movesx,
            movejx,
            set_velx,
            set_accx,
            set_velj,
            set_accj,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_RA_OVERRIDE,
            DR_MVS_VEL_CONST,
            get_external_torque,
            get_current_solution_space,
            drl_script_stop,
            DR_QSTOP,
            DR_TOOL,
            DR_WORLD,
            posx,
            trans,
            set_digital_output,
            get_digital_input,
            wait,
            get_current_posx,
            get_current_posj,
            amovel
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    # 툴과 TCP 설정
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 변수 초기화 (컵의 크기 및 개수 설정)
    global H_, H_top_, num, d_, D_, i_num, start

    start = []
    i_num = 0  # 현재 쌓을 컵의 인덱스
    num = 12  # 총 컵 개수
    d_ = 55  # 컵 상단 지름 
    D_ = 76  # 컵 하단 지름
    H_ = 95  # 컵의 높이
    H_top_ = 88.45 + 10*num + 20  # 초기 높이 설정

    # 힘 감지를 이용한 수직 이동 함수
    def vertical_move_with_force_detection():
        task_compliance_ctrl(stx=[700, 700, 300, 300, 300, 300])
        set_desired_force(fd=[0, 0, -25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        return get_current_posx()
    
    def point_line(x, y, h, n):
        D = 79
        H = 95
        xi = x
        points = []

        for i in range(n + 1):
            if i != 0:
                y += D * math.sqrt(3) / 2
            
            for j in range(n - i):
                if not j == 0:
                    x += 80
                points.append((x, y, h))
            
            x = xi + 40 * (i + 1)

        return points

    def points(x, y, h, n):
        D = 79
        H = 95
        cup_points = []
        
        for i in range(n):
            cup_points += point_line(x + i * 40, y + i * (D * math.sqrt(3) / 6), h + i * H, n - i)
        
        return cup_points
    
    # 물체를 집는 동작 수행 함수
    def pick_obj(x, y):
        global H_top_, H_, start

        #movej([0, 0, 90, 0, 90, 0], radius=200, ra=DR_MV_RA_OVERRIDE)
        (_, _, _, rx, ry, rz), sol = get_current_posx()
        print(f'sol = {sol}')
        movejx(posx(x, y, H_top_, rx, ry, rz), sol=sol) #ra=DR_MV_RA_OVERRIDE, 컵 12개 전부 쌓여있을 때의 높이로 이동
        wait(0.5)
        
        new_pos, _ = vertical_move_with_force_detection()   # 컵을 집었을 때 x, y, z값 갱신 - 다음 컵 집을 때 이 위치로 이동
        start = new_pos[:2]
        H_top_ = new_pos[2]
        release_compliance_ctrl()
        
        movejx(posx([0, 0, 5, 0, 0, 0]), mod=DR_MV_MOD_REL, vel=20, acc=10, sol=sol)
        movejx(posx(get_current_posx()[0][:3] + [rx, ry, rz]), sol=sol) # 축 정렬
        joint_open()
        wait(0.5)

        if i_num == len(goal_points):   #마지막 컵 돌리기

            setvel(20, 20)
            setvelj(20, 20)

            movel(posx(get_current_posx()[0][:3] + [90, -90, 90]), vel=80, acc=80)  # 툴 90도 회전
            #_, new_sol = get_current_posx()
            #print(f'new_sol = {new_sol}')
            #sol = new_sol

            movejx(posx([0, -15, -13, 0, 0, 0]), mod=DR_MV_MOD_REL, sol=sol)  # 물체를 집기 위한 높이까지 이동
            #movejx(posx([0, -15, 0, 0, 0, 0]), mod=DR_MV_MOD_REL, sol=sol)

        else : movejx(posx([0, 0, -20, 0, 0, 0]), mod=DR_MV_MOD_REL, sol=sol)   # 물체를 집기 위한 높이까지 이동
        joint_grab()

        
        if i_num == len(goal_points):   #마지막 컵 돌리기

            setvel(20, 20)
            setvelj(20, 20)

            movel(posx([0, 0, 10, 0, 0, 0]), mod=DR_MV_MOD_REL, vel=120, acc=120)  # 상단 이동 - 회전반경 확보

            movel(posx([100, 0, H_, 0, 0, 0]), mod=DR_MV_MOD_REL, vel=120, acc=120)    # x축 이동 - 특이점 회피

            #movel(posx([0, 0, H_, 0, 0, 0]), mod=DR_MV_MOD_REL, vel=60, acc=60)   #올리고
            posj2 = get_current_posj()
            posj2[5] += 180 # 툴 회전
            movej(posj2, vel=120, acc=120)
            wait(1.0)
            joint_open()                                            #회전

        else : 
            movejx(posx([0, 0, H_-10, 0, 0, 0]), mod=DR_MV_MOD_REL, sol=sol)

    # 물체를 목표 위치에 놓는 함수
    def put_obj(x, y, z):
        global i_num
        
        offset = 40
        (_, _, zi, rx, ry, rz), sol = get_current_posx()

        zi = zi if zi>(88.45 + z + offset) else 88.45 + z + offset
        x0 = posx(x, y, zi, rx, ry, rz)
        x1 = posx(x, y, 88.45 + z + offset, rx, ry, rz)
        x2 = posx(x, y, 88.45 + z, rx, ry, rz)

        #movejx(posx(x, y, zi, rx, ry, rz), radius=200, ra=DR_MV_RA_OVERRIDE, sol=sol)
        #movejx(posx(x, y, 88.45 + z + offset, rx, ry, rz), radius=200, ra=DR_MV_RA_OVERRIDE, sol=sol)
        #movejx(posx([0, 0, -offset, 0, 0, 0]), mod=DR_MV_MOD_REL, radius=200, ra=DR_MV_RA_OVERRIDE, v=30, a=30, sol=sol)
        movesx([x0, x1, x2], vel=[200, 200], acc=[200, 200], vel_opt=DR_MVS_VEL_CONST)
        wait(0.3)

        joint_open()

        if not i_num == len(goal_points) : 
            movejx(posx([0, 0, offset, 0, 0, 0]), mod=DR_MV_MOD_REL,sol=sol)
            joint_close()

        else : movejx(posx([0, 0, -50, 0, 0, 0]), mod=DR_MV_MOD_REL,sol=sol)

        i_num += 1

    ###main_code####

    initialx = [417.86, 210.83, 212.20, 12.62, 180, -180]   # 시작 위치
    goalx = [289.94, -217.14, 89.45, 61, 180, -105]         # 목표 위치
    goal_points = points(goalx[0], goalx[1], 0, 3)          # 목표 위치 포인트 계산
    start = initialx[:2]

    while rclpy.ok() and i_num <= len(goal_points):
        if i_num == 0 : 
            start_time = time.time()
            joint_open()
            movej([0, 0, 90, 0, 90, 0], vel=80, acc=80)#, radius=200, ra=DR_MV_RA_OVERRIDE)  # 초기 위치
            joint_close()
        

        #movej([0, 0, 90, 0, 90, 0], vel=80, acc=80)
        #joint_close()
        setvel(20, 20)
        setvelj(60, 60)
        #movej([0, 0, 90, 0, 90, 0], radius=200, ra=DR_MV_RA_OVERRIDE)  # 초기 위치

        pick_obj(*start)

        if i_num == len(goal_points) : #컵 뒤집기
            goal_points[i_num-1] = list(goal_points[i_num-1])
            #goal_points[i_num-1][0] += 5
            goal_points[i_num-1][1] -= 25 #마지막 포인트 - 마지막 컵 위치에서 z +20 상단
            goal_points[i_num-1][2] += 16+H_ #마지막 포인트 - 마지막 컵 위치에서 z +20 상단
            put_obj(*goal_points[i_num-1])

        else : 
            put_obj(*goal_points[i_num])

    end_time = time.time()
    print(f'time = {end_time - start_time}')
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()



