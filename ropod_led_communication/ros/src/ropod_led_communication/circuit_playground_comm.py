import serial
import rospy

class CircuitPlaygroundComm(object):
    def __init__(self):
        try:
            self.serial = serial.Serial('/dev/circuit_playground', 9600, 5)
        except:
            rospy.logerr("Could not connect to Circuit Playground")
        self.YELLOW = "120 120 0"
        self.RED = "120 0 0"
        self.GREEN = "0 120 0"
        self.BLUE = "0 120 120"
        self.WHITE = "120 120 120"

    def on(self, color):
        # command_id r g b
        command = "1 " + color
        self.__send_command(command)

    def off(self):
        command = "0"
        self.__send_command(command)

    def blink_and_buzz(self, color):
        # command_id speed r g b
        command = "5 200 " + color
        self.__send_command(command)

    def blink_forever(self, color):
        # command_id speed r g b
        command = "3 80 " + color
        self.__send_command(command)

    def blink(self, color, num_times):
        # command_id speed r g b
        command = "4 100 " + color + " " + str(num_times)
        self.__send_command(command)

    def blink_on(self, color, num_times):
        # command_id speed r g b
        command = "4 1000 " + color + " " + str(num_times)
        self.__send_command(command)


    def rotate(self, color):
        # command_id dir speed r g b
        command = "2 0 100 " + color
        self.__send_command(command)

    def set_progress(self, progress):
        # command_id progress (in range 0-100 and integer)
        command = "7 " + str(progress)
        self.__send_command(command)

    def blink_quadrants(self, color, quadrants):
        q = ""
        for i in range(4):
            if i in quadrants:
                q += "1 "
            else:
                q += "0 "
        command = "9 100 " + q + color
        self.__send_command(command)

    def set_quadrant(self, color, quadrants):
        q = ""
        for i in range(4):
            if i in quadrants:
                q += "1 "
            else:
                q += "0 "
        command = "8 " + q + color
        self.__send_command(command)


    def __send_command(self, command):
        command += '\n'
        try:
            self.serial.flushInput()
            self.serial.write(command.encode())
        except:
            rospy.logerr("Error sending command to circuit playground")
