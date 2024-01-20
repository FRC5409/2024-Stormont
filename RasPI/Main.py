import NetworkTables
import LED
import time

REFRESH_TIME = 0.05

def Color(red, green, blue):
    return int('{:02x}{:02x}{:02x}'.format(red, green, blue), 16)

while True:
    if (NetworkTables.instanceConnected()):
        lastState = state
        state = NetworkTables.getState()

        arr = NetworkTables.getPrimaryColor()
        primaryColor = Color(arr[0], arr[1], arr[2])

        arr = NetworkTables.getSecondaryColor()
        secondaryColor = Color(arr[0], arr[1], arr[2])

        if lastState != state:
            LED.newAnimation()

        if state == 0:
            LED.setLEDColor(primaryColor)
        elif state == 1:
            LED.blinkLEDs(primaryColor, secondaryColor)
        elif state == 2:
            LED.sinWave(primaryColor, secondaryColor)
        elif state == 3:
            LED.sinFlow(primaryColor, secondaryColor)
        elif state == 4:
            LED.rainbowCycle(primaryColor, secondaryColor)
        elif state == 5:
            LED.rainbowBlink(primaryColor, secondaryColor)
    else:
        # Default Animation
        LED.setLEDColor(Color(255, 0, 0))

    # Render LEDs
    time.sleep(REFRESH_TIME)