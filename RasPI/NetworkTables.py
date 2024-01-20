import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("10.54.9.2") # 10.TE.AM.2
inst.setServer("10.54.9.2") #    10.TE.AM.2

table = inst.getTable("Shuffleboard").getSubTable("LED_COMMUNICATION_TAB")

LEDstate = table.getIntegerTopic("STATE").subscribe(3)
LEDPrimeColor = table.getIntegerArrayTopic("PRIMARY_COLOR").subscribe([242, 242, 5])
LEDSecondColor = table.getIntegerArrayTopic("SECONDARY_COLOR").subscribe([0, 0, 0])

def getState():
    return LEDstate.get()

def getPrimaryColor():
    return LEDPrimeColor.get()

def getSecondaryColor():
    return LEDSecondColor.get()

def instanceConnected():
    return inst.isConnected()
