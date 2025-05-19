import rclpy

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gear", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            amove_periodic,
            move_periodic,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_stiffnessx,
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
            amovel,
            DR_Error
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
        wait_digital_input(1)

    def open_signal():
        wait_digital_input(2)

    def close_signal():
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



    def pick_obj(x, y):
        setvel(100, 100)
        # 해당위치로 이동
        (_, _, z, rx, ry, rz), _ = get_current_posx()
        movel(posx(x, y, 128, rx, ry, rz), mod=DR_MV_MOD_ABS)
        joint_open()
        # 물체 집기고 올리기
        movel(posx(0.0, 0.0, -80, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        joint_grab()
        movel(posx(0.0, 0.0, 80, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)

    def move_obj():
        setvel(100, 100)
        #이동
        pre_pos, _ = get_current_posx()
        movel(posx(0.0, -296.0, 0.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)

    def put_obj():
        move_obj()
        #물체
        movel(posx(0.0, 0.0, -40, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        setvel(100, 30)
        movel(posx(0.0, 0.0, -20, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
        joint_open()
        #놓고 올리기
        setvel(1500, 1000)
        movel(posx(0.0, 0.0, 50, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)
    
    
    def center_gear(x, y):
        # 1. 물체를 집고 이동
        pick_obj(x, y)
        move_obj()
        movel(posx(0.0, 0.0, -50.0, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)

        # 2. 순응 제어 설정
        #amovel(posx(0, 0, -30, 0, 0, 0), v=30, a=30, mod=DR_MV_MOD_REL)
        #task_compliance_ctrl([300, 300, 500, 1500, 1500, 1500], time=0.5)
        #task_compliance_ctrl()
        #set_stiffnessx([300, 300, 500, 1500, 1500, 1500], time=0.5)
        #wait(0.8)
        #set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],  mod=DR_FC_MOD_REL)

        # 3. 기어 조립 작업
        #try:
        while True:
            task_compliance_ctrl()
            set_stiffnessx([300, 300, 500, 1500, 1500, 1500], time=0.5)
            wait(0.8)
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],  mod=DR_FC_MOD_REL)

            current_pos, _ = get_current_posx()
            current_pos_z = current_pos[2]

            print(current_pos_z)
            print(check_force_condition(axis=DR_AXIS_Z, max=20))

            #if not check_force_condition(axis=DR_AXIS_Z, max=20) and current_pos_z > 47:
            if current_pos_z > 47:
                #amove_periodic(amp=[0, 0, 0, 0, 0, 20], period=2.0, atime=0.1, repeat=1)  # 기어 위치를 조정
                move_periodic(amp=[0, 0, 0, 0, 0, 20], period=[0, 0, 0, 0, 0, 1], repeat=1)
                #move_periodic(amp=[0, 0, 0, 0, 0, 30], period=1.0, atime=0.02, repeat=3, ref=DR_TOOL)
            else:
                print("Force condition cleared. Stopping.")
                drl_script_stop(DR_QSTOP)
                break
        #except DR_Error as e:
        #    print("Error occurred during gear centering {}".format(e))
            '''
        finally:
            # 4. 순응 제어 해제
            #force = get_tool_force(DR_TOOL)
            #print("finall_tool_force x = {}, y = {}, z = {}, rx = {}, rt = {}, rz = {}".format(force[0], force[1], force[2], force[3], force[4], force[5]))
            print("finall_pos x = {}, y = {}, z = {}, rx = {}, rt = {}, rz = {}".format(current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5]))
            release_compliance_ctrl()
            print("Compliance control released.")
        '''
    initialj = [14.14, 33.24, 58.02, -0.23, 88.74, 14.43]
    centerx = [572.24, 152.37, 46.93, 0.61, -179.99, 0.40]

    l = -296

    while rclpy.ok():
        setvel(30, 10)

        movej(initialj, v=30, a=30)
        center_gear(centerx[0], centerx[1])
        release_compliance_ctrl()
        joint_open()
        movel(posx(0.0, 0.0, 80, 0.0, 0.0, 0.0), mod=DR_MV_MOD_REL)


        #task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
