from evdev import InputDevice, categorize, ecodes
import math
import threading

gamepad = InputDevice('/dev/input/event0')

ABS_Y = 0
ABS_X = 1
ABS_RY = 3
ABS_RX = 4
ABS_Z = 317
ABS_RZ = 318
BTN_TL = 2
BTN_TR = 5
BTN_SOUTH = 304
BTN_NORTH = 308
BTN_WEST = 307
BTN_EAST = 305
BTN_THUMBL = 310
BTN_THUMBR = 311
BTN_SELECT = 314
BTN_START = 315
BTN_TRIGGER_HAPPY1 = 16 #-1
BTN_TRIGGER_HAPPY2 = 16 #+1
BTN_TRIGGER_HAPPY3 = 17 #-1
BTN_TRIGGER_HAPPY4 = 17 #+1

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

class xboxcontroller(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):  # return the buttons/triggers that you care about in this methode
        # x = self.LeftJoystickX
        # y = self.LeftJoystickY
        x = round(self.LeftJoystickX, 1)
        y = round(self.LeftJoystickY, 1)
        a = self.A
        b = self.Y  # b=1, x=2
        rb = self.RightBumper
        return [x, y, a, b, rb]

    def readall(self):
        return [
            round(self.LeftJoystickY, 1),
            round(self.LeftJoystickX, 1),
            round(self.RightJoystickY, 1),
            round(self.RightJoystickX, 1),
            self.LeftTrigger,
            self.RightTrigger,
            self.LeftBumper,
            self.RightBumper,
            self.A,
            self.X,
            self.Y,
            self.B,
            self.LeftThumb,
            self.RightThumb,
            self.Back,
            self.Start,
            self.LeftDPad,
            self.RightDPad,
            self.UpDPad,
            self.DownDPad
        ]
    
    
    def readallraw(self):
        return [
            self.LeftJoystickY,
            self.LeftJoystickX,
            self.RightJoystickY,
            self.RightJoystickX,
            self.LeftTrigger,
            self.RightTrigger,
            self.LeftBumper,
            self.RightBumper,
            self.A,
            self.X,
            self.Y,
            self.B,
            self.LeftThumb,
            self.RightThumb,
            self.Back,
            self.Start,
            self.LeftDPad,
            self.RightDPad,
            self.UpDPad,
            self.DownDPad
        ]


    def readneeded(self):
        return [
            round(float(self.LeftJoystickY), 1),
            round(float(self.LeftJoystickX), 1),
            #round(self.RightJoystickY, 1),
            #round(self.RightJoystickX, 1),
            #round(self.RightTrigger - self.LeftTrigger, 1)
        ]

    def _monitor_controller(self):
        while True:
            for event in gamepad.read_loop():
                #print(event)
                if event.code == ABS_Y and event.type == 3:  # x & Y Wurden Vertauscht? fixed
                    self.LeftJoystickY = remap(event.value, -32767, 32768, +1, -1) #/ XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == ABS_X:  # s.o.
                    self.LeftJoystickX = remap(event.value, -32767, 32768, +1, -1) #/  #/ XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == ABS_RY:
                    self.RightJoystickY = remap(event.value, -32767, 32768, +1, -1) #/ XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == ABS_RX:
                    self.RightJoystickX = remap(event.value, -32767, 32768, +1, -1) #/ XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == ABS_Z:
                    self.LeftTrigger = event.value # NORMALIZE FEHLT  # normalize between 0 and 1
                elif event.code == ABS_RZ:
                    self.RightTrigger = event.value # NORMALIZE FEHLT  # normalize between 0 and 1
                elif event.code == BTN_TL:
                    self.LeftBumper = event.value
                elif event.code == BTN_TR:
                    self.RightBumper = event.value
                elif event.code == BTN_SOUTH:
                    #print("pressed")
                    #print(event.value)
                    self.A = event.value
                elif event.code == BTN_NORTH:
                    self.Y = event.value
                elif event.code == BTN_WEST:
                    self.X = event.value
                elif event.code == BTN_EAST:
                    self.B = event.value
                elif event.code == BTN_THUMBL:
                    self.LeftThumb = event.value
                elif event.code == BTN_THUMBR:
                    self.RightThumb = event.value
                elif event.code == BTN_SELECT:
                    self.Back = event.value
                elif event.code == BTN_START:
                    self.Start = event.value
                #elif event.code == BTN_TRIGGER_HAPPY1:
                #    self.LeftDPad = event.state
                #elif event.code == BTN_TRIGGER_HAPPY2:
                #    self.RightDPad = event.state
                #elif event.code == BTN_TRIGGER_HAPPY3:
                #    self.UpDPad = event.state
                #elif event.code == BTN_TRIGGER_HAPPY4:
                #    self.DownDPad = event.state
