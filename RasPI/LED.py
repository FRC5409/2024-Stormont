import math
import Main

LED_CHANNEL = 0
LED_COUNT = 60              # How many LEDs to light.
LED_FREQ_HZ = 800000        # Frequency of the LED signal.  Should be 800khz or 400khz.
LED_DMA_NUM = 10            # DMA channel to use, can be 0-14.
LED_GPIO = 18               # GPIO connected to the LED signal line.  Must support PWM!
LED_BRIGHTNESS = 155

BLINK_SPEED = 5          # in REFRESH_TIME intervals

SIN_COUNT = 8
SIN_SPEED = 0.5

SIN_HS = 0.05
SIN_VS = 30.0

RAINBOW_CYCLE_SPEED = 0.5   # negative speed to reverse
RAINBOW_BLINK_SPEEED = 0.5; # negative speed to reverse

def newAnimation():
    LEDTimer = 0.0
    animationTime = 0

def setLEDColor(color):
    return

def setLEDColorAt(color, index):
    return

def blinkLED(primeColor, secondColor):
    global LEDTimer, animationTime

    if LEDTimer <= BLINK_SPEED:
        setLEDColor(primeColor)
    elif LEDTimer <= BLINK_SPEED * 2:
        setLEDColor(secondColor)
    else:
        LEDTimer = 0.0
        animationTime = 0
        return

    LEDTimer += 1

# sinWave LED function (Doesn't actually use the sin function named it after Lex's function)
def sinWave(primeColor, secondColor): 
    global LEDTimer
    LEDTimer += SIN_SPEED

    for i in range(0, LED_COUNT):
        if ((i + LEDTimer) % (SIN_COUNT * 2)) <= SIN_COUNT:
            setLEDColorAt(i, primeColor)
        else:
            setLEDColorAt(i, secondColor)

# sinFlow LED function, alternates the movement in a Sin fasion, closely related to the sineWave LED function
def sinFlow(primeColor, secondColor):
    global LEDTimer, animationTime
    LEDTimer += 1

    animationTime = math.sin(LEDTimer * SIN_HS) * SIN_VS

    # Makes the function smoother
    if (abs(animationTime) >= SIN_VS - 0.5):
        LEDTimer += 2
    
    animationTime = math.floor(animationTime)

    for i in range(-SIN_COUNT, LED_COUNT + SIN_COUNT):
        if (((i + animationTime) % (SIN_COUNT * 2)) <= SIN_COUNT):
            if (i >= 0 & i < LED_COUNT):
                setLEDColorAt(i, primeColor)
        else:
            if (i >= 0 & i < LED_COUNT):
                setLEDColorAt(i, secondColor)

# rainbowCycle LED function is a rainbow on the LEDs that moves based on the specified speed
def rainbowCycle():
    global LEDTimer
    LEDTimer -= RAINBOW_CYCLE_SPEED

    rainbow_colors_rgb = [
        Main.Color(255, 0, 0),      # Red
        Main.Color(255, 69, 0),     # Orange
        Main.Color(255, 255, 0),    # Yellow
        Main.Color(0, 255, 0),      # Green
        Main.Color(0, 0, 255),      # Blue
        Main.Color(138, 43, 226)    # Violet
    ]

    for i in range(LED_COUNT):
            setLEDColorAt(i, rainbow_colors_rgb[int(LEDTimer + i) % len(rainbow_colors_rgb)])

# rainbowBlink LED function cycles through the LEDs and sets the color of all the LEDs to the curret rainbow color
def rainbowBlink():
    global LEDTimer
    LEDTimer += RAINBOW_BLINK_SPEEED

    rainbow_colors_rgb = [
        Main.Color(255, 0, 0),      # Red
        Main.Color(255, 69, 0),     # Orange
        Main.Color(255, 255, 0),    # Yellow
        Main.Color(0, 255, 0),      # Green
        Main.Color(0, 0, 255),      # Blue
        Main.Color(138, 43, 226)    # Violet
    ]

    setLEDColor(rainbow_colors_rgb[(int(LEDTimer)) % len(rainbow_colors_rgb)])