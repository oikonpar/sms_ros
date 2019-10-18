import serial, time, sys, platform


def checksum(cmd, bc, x):
    ret = 0
    ret ^= (cmd ^ bc)
    for i in range(0, bc):
        ret ^= x[i]
    return ret

def getAnalogInResponse():
    success = False
    b = ser.read(6)
    #The first 6 bytes are useless for us (2 header bytes + addressed and owned id + command id and byte count = 6 bytes)
    ain = [0, 0, 0, 0]
    if ord(b[5]) != 4:
        return success,ain
    success = True
    j = 0
    for i in range(0, 4):
        b+=ser.read()
        shift = 0
        ain[i] += (ord(b[j + 6]) & 0xFF) << shift
        j+=1
        b+=ser.read()
        shift = 8
        ain[i] += (ord(b[j + 6]) & 0xFF) << shift
        j+=1
    b += ser.read()
    #necessary for not leaving any garbage in the serial read interface (we have to read the lrc byte)
    return success,ain
def getDigitalIOResponse():
    success = False
    b = ser.read(6)
    dio1 = 0
    dio2 = 0
    dio3 = 0
    #The first 6 bytes are useless for us (2 header bytes + addressed and owned id + command id and byte count = 6 bytes)
    if ord(b[5]) != 1:
        return success,dio1,dio2,dio3
    else:
        success = True
        b+=ser.read()
        dio1 = ord(b[6]) & 0x01
        dio2 = ord(b[6]) & 0x02
        dio3 = ord(b[6]) & 0x04
    b += ser.read()
    #necessary for not leaving any garbage in the serial read interface (we have to read the lrc byte)
    return success,dio1,dio2,dio3

def getResponse():
    carrying_data = False
    b = ser.read(6)
    #The first 6 bytes are useless for us (2 header bytes + addressed and owned id + command id and byte count = 6 bytes)
    num = 0
    for i in range(0, ord(b[5])):
        carrying_data = True
        b+=ser.read()
        shift = i * 8
        num += (ord(b[i + 6]) & 0xFF) << shift
    b += ser.read()
    #necessary for not leaving any garbage in the serial read interface (we have to read the lrc byte)
    return carrying_data,num
def print_message():
    print("Usage: [sudo] python SMSLibrary.py port_number command [Node Id] [value]\n")
    print("Commands are: start [Node Id], stop [Node Id], reset [Node Id], startall, stopall, resetall,\n")
    print("getvel [Node Id], getacc [Node Id], getpos [Node Id],\n")
    print("move [Node Id] [start] [goal], setvel [Node Id] [value], setacc [Node Id] [value], test [Node Id]")

def send8Bytes(mid, cmd_id, value):
    data = {}
    data[7] = (value >> 56) & 0xff
    data[6] = (value >> 48) & 0xff
    data[5] = (value >> 40) & 0xff
    data[4] = (value >> 32) & 0xff
    data[3] = (value >> 24) & 0xff
    data[2] = (value >> 16) & 0xff
    data[1] = (value >> 8) & 0xff
    data[0] = value & 0xff
    lrc = checksum(cmd_id, 8, data)
    command = bytearray(
        [h_0, h_1, mid, '\x01', chr(cmd_id), '\x08', data[0], data[1], data[2], data[3], data[4], data[5], data[6],
         data[7], lrc])
    ser.write(command)

def send4Bytes(mid, cmd_id, value):
    data = {}
    data[3] = (value >> 24) & 0xff
    data[2] = (value >> 16) & 0xff
    data[1] = (value >> 8) & 0xff
    data[0] = value & 0xff
    lrc = checksum(cmd_id, 4, data)
    command = bytearray(
        [h_0, h_1, mid, '\x01', chr(cmd_id), '\x04', data[0], data[1], data[2], data[3], lrc])
    ser.write(command)

def send2Bytes(mid, cmd_id, value):
    data = {}
    data[1] = (value >> 8) & 0xff
    data[0] = value & 0xff
    lrc = checksum(cmd_id, 2, data)
    command= bytearray([h_0, h_1, mid, '\x01', chr(cmd_id), '\x02', data[0], data[1], lrc])
    ser.write(command)

def sendByte(mid, cmd_id, value):
    lrc = checksum(cmd_id, 1, value)
    command= bytearray([h_0, h_1, mid, '\x01', chr(cmd_id), '\x01', value, lrc])
    ser.write(command)

def sendCommand(mid, cmd_id):
    command= bytearray([h_0, h_1, mid, '\x01', chr(cmd_id), '\x00', chr(cmd_id)])
    ser.write(command)


def startall(startId, stopId):
    for i in range(startId, stopId+1):
        start(i)

def stopall(startId, stopId):
    for i in range(startId, stopId+1):
        stop(i)

def resetall(startId, stopId):
    for i in range(startId, stopId+1):
        resetErrors(i)

#-----------------------------------------------------------------------------------------------------------------------
#Important! In Set Commands we return the not getResponse()[0] because getResponse() returns as it's first argument a boolean
# variable, carrying_data, which represents if the response carries any data. Because the response of the Set Commands
# doesn't carry any data, the carrying_data will be False (but the response to the command was successful).
#  So we negate the getResponse()[0] to give as True for these commands.
#-----------------------------------------------------------------------------------------------------------------------

def setPIDgainP(mid, val):
    send2Bytes(mid, 0, val)
    return not getResponse()[0]
def setPIDgainI(mid, val):
    send2Bytes(mid, 1, val)
    return not getResponse()[0]
def setPIDgainD(mid, val):
    send2Bytes(mid, 2, val)
    return not getResponse()[0]
def setProfileAcceleration(mid, val):
    send4Bytes(mid, 3, val)
    return not getResponse()[0]
def setProfileConstantVelocity(mid, val):
    send4Bytes(mid, 4, val)
    return not getResponse()[0]
def setCurrentLimit(mid, val):
    send2Bytes(mid, 5, val)
    return not getResponse()[0]
def setDurationForCurrentLimit(mid, val):
    send2Bytes(mid, 6, val)
    return not getResponse()[0]
def moveWithVelocity(mid, val):
    send4Bytes(mid, 7, val)
    return not getResponse()[0]
def moveToAbsolutePosition(mid, pos):
    send8Bytes(mid, 8, pos)
    return not getResponse()[0]
def moveToRelativePosition(mid, pos):
    send8Bytes(mid, 9, pos)
    return not getResponse()[0]
def profiledMoveWithVelocity(mid, val):
    send4Bytes(mid,10, val)
    return not getResponse()[0]
def profiledMoveToAbsolutePosition(mid, pos):
    send8Bytes(mid, 11, pos)
    return not getResponse()[0]
def profiledMoveToRelativePosition(mid, pos):
    send8Bytes(mid, 12, pos)
    return not getResponse()[0]
def setVelocitySetpoint(mid, val):
    send4Bytes(mid, 13, val)
    return not getResponse()[0]
def setAbsolutePositionSetpoint(mid, pos):
    send8Bytes(mid, 14, pos)
    return not getResponse()[0]
def setRelativePositionSetpoint(mid, pos):
    send8Bytes(mid, 15, pos)
    return not getResponse()[0]
def setProfiledVelocitySetpoint(mid, val):
    send4Bytes(mid, 16, val)
    return not getResponse()[0]
def setProfiledAbsolutePositionSetpoint(mid, pos):
    send8Bytes(mid, 17, pos)
    return not getResponse()[0]
def setProfiledRelativePositionSetpoint(mid, pos):
    send8Bytes(mid, 18, pos)
    return not getResponse()[0]
def configureDigitalIOs(mid, dio1, dio2, dio3):
    data = {}
    data[0] = 0
    if (dio1):
        data[0] |= 0x01
    if (dio2):
        data[0] |= 0x02
    if (dio3):
        data[0] |= 0x04
    sendByte(mid, 19, data[0])
    return not getResponse()[0]

def setDigitalOutputs(mid, dio1, dio2, dio3):
    data = {}
    data[0] = 0
    if (dio1):
        data[0] |= 0x01
    if (dio2):
        data[0] |= 0x02
    if (dio3):
        data[0] |= 0x04
    sendByte(mid, 20, data[0])
    return not getResponse()[0]

def setNodeID(oldNodeId, newNodeId):
    data = {}
    data[0] = newNodeId
    lrc = checksum(21, 1, data)

    command = bytearray(['\x55', '\xAA', oldNodeId, '\x01', '\x15', '\x01', data[0], lrc])
    ser.write(command)
    return not getResponse()[0]

def resetIncrementalPosition(mid):
    sendCommand(mid, 24)
    return not getResponse()[0]

def start(mid):
    sendCommand(mid, 25)
    # time.sleep(0.02)
    return not getResponse()[0]

def halt(mid):
    sendCommand(mid, 26)
    # time.sleep(0.02)
    return not getResponse()[0]

def stop(mid):
    sendCommand(mid, 27)
    # time.sleep(0.02)
    return not getResponse()[0]

def resetErrors(mid):
    sendCommand(mid, 30)
    # time.sleep(0.02)
    return not getResponse()[0]

def getPIDgainP(mid):
    sendCommand(mid, 100)
    return getResponse()

def getPIDgainI(mid):
    sendCommand(mid, 101)
    return getResponse()

def getPIDgainD(mid):
    sendCommand(mid, 102)
    return getResponse()

def getProfileAcceleration(mid):
    sendCommand(mid, 103)
    return getResponse()

def getProfileConstantVelocity(mid):
    sendCommand(mid, 104)
    return getResponse()

def getCurrentLimit(mid):
    sendCommand(mid, 105)
    return getResponse()

def getCurrentLimitDuration(mid):
    sendCommand(mid, 106)
    return getResponse()

def getDigitalIOConfiguration(mid, dio):
    sendCommand(mid, 107)
    return getDigitalIOResponse()

def getDigitalIn(mid, din):
    sendCommand(mid, 109)
    return getDigitalIOResponse()

def getAnalogIn(mid, ain):
    sendCommand(mid, 110)
    return getAnalogInResponse()

def getPosition(mid):
    sendCommand(mid, 111)
    return getResponse()

def getAbsolutePosition(mid):
    sendCommand(mid, 112)
    return getResponse()

def getVelocity(mid):
    sendCommand(mid, 113)
    return getResponse()

def getCurrent(mid):
    sendCommand(mid, 114)
    return getResponse()

def broadCastDoMove():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xC8, 0x00, 0xC8])
    ser.write(command)
    return getResponse()

def broadcastStart():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xC9, 0x00, 0xC9])
    ser.write(command)
    return True

def broadcastHalt():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xCA, 0x00, 0xCA])
    ser.write(command)
    return True

def broadcastStop():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xCB, 0x00, 0xCB])
    ser.write(command)
    return True

def init(port):
    global ser, h_0, h_1
    h_0 = '\x55'
    h_1 = '\xAA'
    port_string = ''
    if(platform.system()=="Windows"):
        port_string = 'COM%d'%port
    elif(platform.system()=="Linux"):
        port_string = '/dev/ttyUSB%s'%port
    ser = serial.Serial(port_string, 57600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)


def shut_down(port):
    global ser
    if ser.isOpen():
        ser.close()
    exit(0)

def main():

    if (len(sys.argv) < 3):
        print("Too few input arguments.\n")
        print_message()
        if(len(sys.argv)<2):
            exit(0)
        else:
            shut_down(int(sys.argv[1]))
    elif (len(sys.argv)>5):
        print("Too many input arguments.\n")
        print_message()
        shut_down(int(sys.argv[1]))

    init(int(sys.argv[1]))

    if (sys.argv[2] == "getvel"):
        ret = getVelocity(int(sys.argv[3]))
        for b in ret:
            print(ord(b))
        shut_down(int(sys.argv[1]))

    if (sys.argv[2] == "getpos"):
        mid = int(sys.argv[3])
        pos = getPosition(mid)
        print(pos[0],float(pos[1]))
        shut_down(int(sys.argv[1]))


    if (sys.argv[2] == "getacc"):
        ret = getProfileAcceleration(int(sys.argv[3]))
        for b in ret:
            print(ord(b))
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "setvel"):
        setProfileConstantVelocity(int(sys.argv[3]), int(sys.argv[4]))
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "setacc"):
        setProfileAcceleration(int(sys.argv[3]), int(sys.argv[4]))
        shut_down(int(sys.argv[1]))

    if (sys.argv[2] == "startall"):
        startall(0,10)
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "start"):
        start(int(sys.argv[3]))
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "stopall"):
        stopall(0,10)
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "stop"):
        stop(int(sys.argv[3]))
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "resetall"):
        resetall(0,10)
        shut_down(int(sys.argv[1]))
    if (sys.argv[2] == "reset"):
        resetErrors(int(sys.argv[3]))
        shut_down(int(sys.argv[1]))

    if (sys.argv[2] == "move"):
        profiledMoveToAbsolutePosition(int(sys.argv[3]), int(sys.argv[4]))
        time.sleep(0.02)
        shut_down(int(sys.argv[1]))

    if (sys.argv[2] == "test"):
        angle = 0
        direction = 1
        step = 10
        while 1:

            # move(5, 1722*angle)
            # time.sleep(0.01)
            profiledMoveToAbsolutePosition(int(sys.argv[3]), angle)

            print(angle)
            angle = angle + direction * step
            if int(angle) == 0:
                direction = 1
            if int(angle) == 15000:
                direction = -1
            time.sleep(0.03)

if __name__ == '__main__':
    main()
