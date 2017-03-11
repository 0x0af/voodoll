#!/usr/bin/python

import avango.daemon
import os

### functions ###

## Initializes AR Track
def init_art_tracking():

    # create instance of DTrack
    _dtrack = avango.daemon.DTrack()
    _dtrack.port = "5000" # ART port

    _dtrack.stations[10] = avango.daemon.Station('tracking-art-glasses-1') # 3D-TV wired shutter glasses
    _dtrack.stations[2] = avango.daemon.Station('tracking-art-glasses-2') # small powerwall polarization glasses

    _dtrack.stations[11] = avango.daemon.Station('tracking-art-pointer-1') # ednet pointer
    _dtrack.stations[1] = avango.daemon.Station('tracking-art-pointer-2') # AUGUST pointer
    _dtrack.stations[18] = avango.daemon.Station('tracking-art-pointer-3') # gyromouse

    #_dtrack.stations[4] = avango.daemon.Station('tracking-art-prop-1') # LHT1 prop
    #_dtrack.stations[12] = avango.daemon.Station('tracking-art-prop-2') # black-cube prop    


    device_list.append(_dtrack)
    print("ART Tracking started")


def init_pst_tracking():

    # create instance of DTrack
    _pst = avango.daemon.DTrack()
    _pst.port = "5020" # PST port

    _pst.stations[1] = avango.daemon.Station('tracking-pst-glasses-1')
    _pst.stations[2] = avango.daemon.Station('tracking-pst-pointer-1') # August pointer

    #_pst.stations[3] = avango.daemon.Station('tracking-pst-prop-1') # LHT1 prop

    device_list.append(_pst)

    print("PST Tracking started!")


def init_spacemouse():

    # search for new spacemouse (blue LED)
    _string = get_event_string(1, "3Dconnexion SpaceNavigator")

    if len(_string) == 0:
        _string = get_event_string(1, "3Dconnexion SpaceNavigator for Notebooks")

    if len(_string) > 0: # new spacemouse was found
        _string = _string.split()[0]

        _spacemouse = avango.daemon.HIDInput()
        _spacemouse.station = avango.daemon.Station('gua-device-spacemouse') # create a station to propagate the input events
        _spacemouse.device = _string
        _spacemouse.timeout = '14' # better !
        #_spacemouse.norm_abs = 'True'

        # map incoming spacemouse events to station values
        _spacemouse.values[0] = "EV_REL::REL_X"   # trans X
        _spacemouse.values[1] = "EV_REL::REL_Z"   # trans Y
        _spacemouse.values[2] = "EV_REL::REL_Y"   # trans Z
        _spacemouse.values[3] = "EV_REL::REL_RX"  # rotate X
        _spacemouse.values[4] = "EV_REL::REL_RZ"  # rotate Y
        _spacemouse.values[5] = "EV_REL::REL_RY"  # rotate Z

        # buttons
        _spacemouse.buttons[0] = "EV_KEY::BTN_0" # left button
        _spacemouse.buttons[1] = "EV_KEY::BTN_1" # right button

        device_list.append(_spacemouse)
        print("New SpaceMouse started at:", _string)

        return


    print("SpaceMouse NOT found!")
    

def init_keyboard():

    keyboard_name = os.popen("ls /dev/input/by-id | grep \"-event-kbd\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

    keyboard_name = keyboard_name.split()

    for i, name in enumerate(keyboard_name):

        keyboard = avango.daemon.HIDInput()
        keyboard.station = avango.daemon.Station('gua-device-keyboard' + str(i))
        keyboard.device = "/dev/input/by-id/" + name

        keyboard.buttons[0] = "EV_KEY::KEY_W"
        keyboard.buttons[1] = "EV_KEY::KEY_A"
        keyboard.buttons[2] = "EV_KEY::KEY_S"
        keyboard.buttons[3] = "EV_KEY::KEY_D"
        keyboard.buttons[4] = "EV_KEY::KEY_LEFT"
        keyboard.buttons[5] = "EV_KEY::KEY_RIGHT"
        keyboard.buttons[6] = "EV_KEY::KEY_UP"
        keyboard.buttons[7] = "EV_KEY::KEY_DOWN"
        keyboard.buttons[8] = "EV_KEY::KEY_Q"
        keyboard.buttons[9] = "EV_KEY::KEY_E"
        keyboard.buttons[10] = "EV_KEY::KEY_PAGEUP"
        keyboard.buttons[11] = "EV_KEY::KEY_PAGEDOWN"
        keyboard.buttons[12] = "EV_KEY::KEY_KPPLUS"
        keyboard.buttons[13] = "EV_KEY::KEY_KPMINUS"
        keyboard.buttons[14] = "EV_KEY::KEY_SPACE"
        keyboard.buttons[15] = "EV_KEY::KEY_LEFTCTRL"
        keyboard.buttons[16] = "EV_KEY::KEY_1"
        keyboard.buttons[17] = "EV_KEY::KEY_2"
        keyboard.buttons[18] = "EV_KEY::KEY_3"
        keyboard.buttons[19] = "EV_KEY::KEY_4"
               

        device_list.append(keyboard)
        print("Keyboard " + str(i) + " started at:", name)



## Initalizes a mouse.
def init_mouse():

    ## search keyboard xy
    _string = get_event_string(1, "Logitech USB-PS/2 Optical Mouse")

    if len(_string) == 0:
        _string = get_event_string(1, "Logitech USB Optical Mouse")

    if len(_string) == 0:
        _string = get_event_string(1, "Dell Dell USB Optical Mouse")
        
    if len(_string) > 0: # some mouse found
        _string = _string.split()[0]

        # create a station to propagate the input events
        _mouse = avango.daemon.HIDInput()
        _mouse.station = avango.daemon.Station("gua-device-mouse")
        _mouse.device = _string
        #_mouse.timeout = '30'
        _mouse.timeout = '10'

        _mouse.values[0] = "EV_REL::REL_X"
        _mouse.values[1] = "EV_REL::REL_Y"

        _mouse.buttons[0] = "EV_KEY::BTN_LEFT"
        _mouse.buttons[1] = "EV_KEY::BTN_RIGHT"
        _mouse.buttons[2] = "EV_KEY::BTN_MIDDLE"

        device_list.append(_mouse)

        #os.system("xinput --set-prop keyboard:'Logitech USB-PS/2 Optical Mouse' 'Device Enabled' 0") # disable X-forwarding of events

        print("Mouse started at:", _string)

    else:
        print("Mouse NOT found!")


def init_pointer1(): # orange pointer
    _string = get_event_string(1, "MOSART Semi. Input Device")
    _string = _string.split()

    if len(_string) > 0:
        _string = _string[0]
        
        _pointer = avango.daemon.HIDInput()
        _pointer.station = avango.daemon.Station("device-pointer-1") # create a station to propagate the input events
        _pointer.device = _string

        _pointer.buttons[0] = "EV_KEY::KEY_PAGEUP"
        _pointer.buttons[1] = "EV_KEY::KEY_B"

        device_list.append(_pointer)

        print("Pointer1 started at:", _string)

        return


    print("Pointer1 NOT found!")


def init_pointer2(): # August pointer
    _string = get_event_string(1, "MOUSE USB MOUSE")
    _string = _string.split()

    if len(_string) > 0:
        _string = _string[0]
        
        _pointer = avango.daemon.HIDInput()
        _pointer.station = avango.daemon.Station("device-pointer-2") # create a station to propagate the input events
        _pointer.device = _string

        _pointer.buttons[0] = "EV_KEY::KEY_PAGEDOWN"
        _pointer.buttons[1] = "EV_KEY::KEY_PAGEUP"

        device_list.append(_pointer)

        print("Pointer2 started at:", _string)

        return


    print("Pointer2 NOT found!")
    


def init_pointer3(): # Gyromouse
    _string = get_event_string(2, "Gyration Gyration RF Technology Receiver")
    _string = _string.split()

    if len(_string) > 0:
        _string = _string[0]
        
        _pointer = avango.daemon.HIDInput()
        _pointer.station = avango.daemon.Station("device-pointer-3") # create a station to propagate the input events
        _pointer.device = _string

        _pointer.buttons[0] = "EV_KEY::KEY_F14"

        device_list.append(_pointer)

        print("Pointer3 started at:", _string)

        return


    print("Pointer3 NOT found!")



## Gets the event string of a given input device.
# @param STRING_NUM Integer saying which device occurence should be returned.
# @param DEVICE_NAME Name of the input device to find the event string for.
def get_event_string(STRING_NUM, DEVICE_NAME):

    # file containing all devices with additional information
    _device_file = os.popen("cat /proc/bus/input/devices").read()
    _device_file = _device_file.split("\n")
    
    DEVICE_NAME = '\"' + DEVICE_NAME + '\"'
    
    # lines in the file matching the device name
    _indices = []

    for _i, _line in enumerate(_device_file):
        if DEVICE_NAME in _line:
            _indices.append(_i)

    # if no device was found or the number is too high, return an empty string
    if len(_indices) == 0 or STRING_NUM > len(_indices):
        return ""

    # else captue the event number X of one specific device and return /dev/input/eventX
    else:
        _event_string_start_index = _device_file[_indices[STRING_NUM-1]+4].find("event")
                
        return "/dev/input/" + _device_file[_indices[STRING_NUM-1]+4][_event_string_start_index:].split(" ")[0]



device_list = []

init_art_tracking()
init_pst_tracking()

init_spacemouse()
init_keyboard()
init_mouse()

init_pointer1()
init_pointer2()
init_pointer3()

avango.daemon.run(device_list)
