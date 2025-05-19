import rclpy
import math
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
            set_velx,
            set_accx,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_AXIS_Z,
            DR_BASE,
            get_tool_force,
            get_external_torque,
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
            amovel
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    global H_, H_top_, num, d_, D_, i_num
    i_num = 0
    num = 12
    d_ = 55
    D_ = 76
    H_ = 95
    #H_top_ = 89.45 + H_ + num*10
    H_top_ = 60.45 + H_ + num*10

    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def setvel(v,a):
        set_velx(v)
        set_accx(a)

    def grab_signal():
        #wait_digital_input(1, 1)
        #wait_digital_input(2, 0)
        wait_digital_input(1)

    def open_signal():
        #wait_digital_input(1, 0)
        #wait_digital_input(2, 1)
        wait_digital_input(2)

    def close_signal():
        #wait_digital_input(1, 1)
        #wait_digital_input(2, 1)
        wait_digital_input(1)
        wait_digital_input(2)

    def joint_open():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        open_signal()

    def joint_close():
        set_digital_output(1, 1)
        set_digital_output(2, 1)
        close_signal()

    def joint_grab():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        grab_signal()

    def vertical_move_with_force_detection():
        setvel(60, 30)

        task_compliance_ctrl(stx=[700, 700, 700, 300, 300, 300])
        set_desired_force(fd=[0, 0, -25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        return get_current_posx()
    
    def point_line(x, y, h, n):
        D = 76
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
        D = 76
        H = 95
        cup_points = []
        
        for i in range(n):
            cup_points += point_line(x + i * 40, y + i * (D * math.sqrt(3) / 6), h + i * H, n - i)
        
        return cup_points

    def pick_obj(x, y):
        global H_top_, H_
        setvel(150, 80)

        joint_close()
        (_, _, _, rx, ry, rz), _ = get_current_posx()
        movel(posx(x, y, H_top_, rx, ry, rz))

        posn, _ = vertical_move_with_force_detection()
        release_compliance_ctrl()
        setvel(150, 80)
    
        movel(posx([0, 0, 5, 0, 0, 0]), mod=DR_MV_MOD_REL)
        joint_open()
        movel(posx([0, 0, -20, 0, 0, 0]), mod=DR_MV_MOD_REL)
        joint_grab()
        movel(posx([0, 0, H_-10, 0, 0, 0]), mod=DR_MV_MOD_REL)

    def put_obj(x, y, z):
        global i_num
        setvel(150, 80)

        (_, _, zi, rx, ry, rz), _ = get_current_posx()

        zi = zi if zi>(89.45 + z + 100) else 89.45 + z + 100
        movel(posx(x, y, zi, rx, ry, rz))
        movel(posx(x, y, 89.45 + z + 100, rx, ry, rz))
        movel(posx([0, 0, -100, 0, 0, 0]), mod=DR_MV_MOD_REL)

        joint_open()
        movel(posx([0, 0, 100, 0, 0, 0]), mod=DR_MV_MOD_REL)

        i_num += 1

    ###main_code####

    initialx = [417.86, 210.83, 212.20, 12.62, 180, -180]
    goalx = [189.94, -217.14, 89.45, 61, 180, -105]
    goal_points = points(goalx[0], goalx[1], 0, 3)

    while rclpy.ok() and i_num < 10:
        joint_open()
        
        movej([0, 0, 90, 0, 90, 0], v=30, a=30)
        posi, _ = get_current_posx()
        setvel(150, 80)

        pick_obj(initialx[0], initialx[1])
        put_obj(goal_points[i_num][0], goal_points[i_num][1], goal_points[i_num][2])

        
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()



