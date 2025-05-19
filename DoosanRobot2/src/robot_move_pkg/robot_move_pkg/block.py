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
    node = rclpy.create_node("block", namespace=ROBOT_ID)
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
            set_digital_output,
            get_digital_input,
            wait,
            get_current_posx,
            amovel
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
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

    def generate_grid(start_x, start_y, x_step=51, y_step=-51, rows=3, cols=3):
        grid = []
        for i in range(rows):
            for j in range(cols):
                x = start_x + i * x_step
                y = start_y + j * y_step
                grid.append((x, y))
        return grid

    def is_force_detected():
        force = get_tool_force(DR_BASE)
        return force[2]

    def vertical_move_with_force_detection():
        setvel(30, 10)
        #amovel(posx(0.0, 0.0, -120.0, 0.0, 0.0, 0.0), ref=DR_BASE, mod=DR_MV_MOD_REL)

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        return get_current_posx()

    def pick_obj(xi, yi):
        setvel(100, 100)
        joint_close()
        (x, y, z, rx, ry, rz), _ = get_current_posx()
        print(f"current pos = {get_current_posx()}")
        print(f'xi = {xi}, yi = {yi}, z = {z}')
        movel(posx(xi, yi, z, rx, ry, rz), mod=DR_MV_MOD_ABS)
        movel(posx(0.0, 0.0, -90.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        pos, _ = vertical_move_with_force_detection()
        release_compliance_ctrl()
        z = pos[2]

        # grab 후 올라감
        setvel(100, 100)
        # 살짝 올리고 joint open(tool 마모 방지)
        movel(posx(0.0, 0.0, 3.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        joint_open()
        movel(posx(0.0, 0.0, -23.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        joint_grab()
        movel(posx(0.0, 0.0, 103.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        # return blcok 높이
        return z

    def move_obj(z, end):
        global Global_long, Global_mid, Global_short
        if z>=73.00 :
            index = 2 + 3*Global_long
            Global_long = Global_long+1

        elif z>=63.00 and z<73.00 :
            index = 1 + 3*Global_mid
            Global_mid = Global_mid + 1

        elif z<63.00 :
            index = 0 + 3*Global_short
            Global_short = Global_short + 1

        print(z)

        setvel(100, 100)
        x = end[index][0]
        y = end[index][1]
        movel(posx(x, y, 180.0, 180.0, -180.0, 180.0), mod=DR_MV_MOD_ABS)

    def put_obj(z):
        h = 200
        # block 높이만큼 하강
        movel(posx(0.0, 0.0, z-h+20, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        setvel(30, 30)
        movel(posx(0.0, 0.0, -20, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        joint_open()
        setvel(100, 100)
        movel(posx(0.0, 0.0, -z+h, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)

    def pick_and_put(end, xi, yi):
        global i_num

        z = pick_obj(xi, yi)
        move_obj(z, end)
        put_obj(z)
        i_num += 1



    intial = [22.62, 1.85, 91.11, -0.05, 87.04, 24.51]
    startj = [22.52, 4.98, 105.13, -0.07, 69.89, 24.44]
    startl = [348.07, 153.30, 73.58, 147.54, -180.00, 149.10]

    starti = (348.07, 154.05)
    endi = (starti[0], starti[1]-148)

    start = generate_grid(starti[0], starti[1])
    end = generate_grid(endi[0], endi[1])

    
    while rclpy.ok() and i_num < 9:
        setvel(50, 10)

        xi = start[i_num][0]
        yi = start[i_num][1]

        movej(intial, vel=30, acc=30)
        pick_and_put(end, xi, yi)
        
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
