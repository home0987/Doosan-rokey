import rclpy
import threading
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup_stacking", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    
    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_digital_output,
            get_digital_input,
            set_desired_force,
            get_current_posx,
            get_current_posj,
            set_tool,
            set_tcp,
            movej,
            movel,
            mwait,
            wait,
            DR_BASE,            
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
        
    h = 95
    w = 77

    #initial = posx([567.86, 210.83, 242.20, 12.62, 180, -180])
    initial = posx([417.86, 210.83, 242.20, 12.62, 180, -180])
    goal = posx([189.94, -217.14, 89.45, 61, 180, 90])

    
    set_tool("Tool Weight_RG2")
    set_tcp("RG2_TCP")

    def check_force():
        movel(posx([417.86, 210.83, 242.20, 12.62, 180, 90]), vel = 60, acc = 60)
        joint_close()
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, max=5):
                pos, _ = get_current_posx()
                if 103 <= pos[2] <= 106:
                    release_compliance_ctrl()
                    movel(posx(0.0, 100.0, 100.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL, vel = 60, acc = 60)
                    release()
                    # movej(posj(0.0, 0.0, 0.0, 0.0, 0.0, 180.0), mod=DR_MV_MOD_REL, vel = 10, acc = 10)
                    movej(posj(52.18, 61.72, 60.2, -52.21, 125.76, 48.33), vel = 10, acc = 10)
                    grip()
                    mwait()
                    movel(posx(0.0, 0.0, 105, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL, vel = 60, acc = 60)
                    mwait()
                    movej(posj(0.0, 0.0, 0.0, 0.0, 0.0, 180.0), mod=DR_MV_MOD_REL, vel = 10, acc = 10)

                else:
                    release_compliance_ctrl()
                    mwait()
                    release()
                    movel(posx(0.0, 0.0, 3.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL, vel = 60, acc = 60)
                    movel(posx(0.0, 0.0, -18.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL, vel = 60, acc = 60)
                    grip()
                    mwait()
                    movel(posx(0.0, 0.0, 105, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL, vel = 60, acc = 60)
                break

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        wait_digital_input(2)

    def grip():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait_digital_input(1)

    def joint_close():
        set_digital_output(1, 1)
        set_digital_output(2, 1)
        wait_digital_input(1)
        wait_digital_input(2)

    def move():
        #---------------------------------- first floor ----------------------------------
        release()
        movej(posj([0, 0, 90, 0, 90, 0]), vel = 60, acc = 60)
        check_force()
        movel(posx([189.94, -217.14, 89.45 + 100, 61, 180, -105]), vel = 60, acc = 60) # 1
        movel(goal, vel = 60, acc = 60) # 1
        release()
        wait(0.5)
        check_force()
        movel(posx([189.94 + w, -217.14, 89.45 + 100, 61, 180, -105]), vel = 60, acc = 60) # 2
        movel(posx([189.94 + w, -217.14, 89.45, 61, 180, -105]), vel = 60, acc = 60)
        release()
        check_force()
        movel(posx([189.94 + (2*w), -217.14, 89.45 + 100, 61, 180, -105]), vel = 60, acc = 60) # 3
        movel(posx([189.94 + (2*w), -217.14, 89.45, 61, 180, -105]), vel = 60, acc = 60)
        release()
        check_force()
        movel(posx([189.94 + (w/2), -217.14 + w, 89.45 + 100, 61, 180, -105]), vel = 60, acc = 60) # 4
        movel(posx([189.94 + (w/2), -217.14 + w, 89.45, 61, 180, -105]), vel = 60, acc = 60)
        release()
        check_force()
        movel(posx([189.94 + (w/2) + w, -217.14 + w, 89.45 + 100, 61, 180, -105]), vel = 60, acc = 60) # 5
        movel(posx([189.94 + (w/2) + w, -217.14 + w, 89.45, 61, 180, -105]), vel = 60, acc = 60)
        release()
        check_force()
        movel(posx([189.94 + w, -217.14 + (2*w), 89.45 + 100, 61, 180, -105]), vel = 60, acc = 60) # 6
        movel(posx([189.94 + w, -217.14 + (2*w), 89.45, 61, 180, -105]), vel = 60, acc = 60)
        release()
        #---------------------------------- first floor ----------------------------------

        #---------------------------------- second floor ----------------------------------
        check_force()
        movel(posx([189.94 + (w/2), -217.14 + (w/2), 89.45 + 200, 61, 180, -105]), vel = 60, acc = 60) # 7
        movel(posx([189.94 + (w/2), -217.14 + (w/2), 89.45 + h, 61, 180, -105]), vel = 60, acc = 60)
        release()
        check_force()
        movel(posx([189.94 + (w/2) + w, -217.14 + (w/2), 89.45 + 200, 61, 180, -105]), vel = 60, acc = 60) # 8
        movel(posx([189.94 + (w/2) + w, -217.14 + (w/2), 89.45 + h, 61, 180, -105]), vel = 60, acc = 60)
        release()
        check_force()
        movel(posx([189.94 + w, -217.14 + (w/2) + w, 89.45 + 200 , 61, 180, -105]), vel = 60, acc = 60) # 9
        movel(posx([189.94 + w, -217.14 + (w/2) + w, 89.45 + h, 61, 180, -105]), vel = 60, acc = 60)
        release()
        #---------------------------------- second floor ----------------------------------

        #---------------------------------- third floor ----------------------------------
        check_force()
        movel(posx([189.94 + w, -217.14 + w, 89.45 + 300 , 61, 180, -105]), vel = 60, acc = 60) # 10
        movel(posx([189.94 + w, -217.14 + w, 89.45 + (2*h), 61, 180, -105]), vel = 60, acc = 60)
        release()
        #---------------------------------- third floor ----------------------------------

        #---------------------------------- final ----------------------------------
        check_force()
        movel(posx([189.94 + w, -217.14 + w, 89.45 + 300 , 90.0, -96.71, -90.53]), vel = 60, acc = 60) # 11
        mwait()
        pos_set = get_current_posj()
        pos_set[3] += -5
        movej(pos_set, vel = 60, acc = 60)
        pos_put, _ = get_current_posx()
        pos_put[2] += -125
        movel(pos_put, vel = 60, acc = 60)

        release()
        #---------------------------------- final ----------------------------------

    if rclpy.ok():
        move()

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()