import rclpy

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


global i_num, Global_long, Global_mid, Global_short
i_num = 0
Global_long = 0
Global_mid = 0
Global_short = 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("jenga", namespace=ROBOT_ID)
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

    def block_point(first_blockx, num, height=16):
        point = first_blockx
        point[2] = first_blockx[2] + (num-1)*height
        return posx(point)
    ###main_code####


    num = 4
    height = 16
    first_blockx = [579.51, -217.40, 10.81, 90, -135.00, -90.00]
    waypoint1 = [579.48, -217.39, 161.80, 55.00, -180, -125.10]
    waypoint2 = [579.48, 124.68, 161.80, 55, 180, -125.45]
    waypoint3 = [579.48, 124.68, 161.80, 90, 135, -90]

    while rclpy.ok():
        setvel(50, 10)

        joint_open()
        movej([0, 0, 90, 0, 90, 0], v=30, a=30)
        block_4 = block_point(first_blockx, 4)

        movel(block_4)
        joint_grab()
        #movesx([waypoint1, waypoint2, waypoint3])
        movel(posx([0, 0, 50, 0, 0, 0]), mod=DR_MV_MOD_REL)
        movel(posx([0, 341, 0, 0, 0, 0]), mod=DR_MV_MOD_REL)
        #movel(posx([0, 0, 0, 0, 30, 0]), mod=DR_MV_MOD_REL, ref=DR_BASE)
        pos, _ = get_current_posx()
        movel(posx(pos[0], pos[1], pos[2], 90, 135, -90)) #, mod=DR_MV_MOD_REL, ref=DR_WORLD)
        movel([0, 0, -30, 0, 0, 0], mod=DR_MV_MOD_REL)

        joint_open()

        
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()



