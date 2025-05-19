# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

try:
    from DSR_ROBOT2 import (
        set_velx,
        set_accx,
        set_velj,
        set_accj,
        set_digital_output,
        get_digital_input,
        wait,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")

        
def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass

def setvel(v,a):
    set_velx(v)
    set_accx(a)

def setvelj(v, a):
    set_velj(v)
    set_accj(a)

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
    
if __name__ == "__main__":
    print('?')
    #main()



