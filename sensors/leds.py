from wpilib import AddressableLED, PowerDistribution
import math, config


class ALeds():
    """
    Addressable LEDS from PWM RIO
    """

    m_led: AddressableLED

    # active_mode: dict
    # speed: int
    # brightness: float
    # last_active_mode: dict
    # last_speed: int
    # last_brightness: float
    # m_ledBuffer: AddressableLED.LEDData

    def __init__(self, id: int, size: int):
        self.size = size
        self.id = id
        self.speed = 5
        self.track_index = 0
        self.blink_index = 0
        self.active_mode = None
        self.last_active_mode = None
        self.last_brightness = None
        self.last_speed = None
        self.brightness = 1

    def init(self):
        self.m_rainbowFirstPixelHue = 0
        self.m_led = AddressableLED(self.id)
        self.m_led.setLength(self.size)
        self.m_ledBuffer = self.m_led.LEDData()
        self.array = [self.m_ledBuffer for i in range(self.size)]
        # for i in range(self.size):
        #     self.array.append(self.m_led.LEDData())
        self.m_led.setData(self.array)
        self.m_led.start()

    def enable(self):
        self.m_led.start()

    def disable(self):
        self.m_led.stop()

    def setBrightness(self, brightness: float):
        self.brightness = brightness

    def getArray(self):
        return [self.m_led.LEDData() for i in range(self.size)].copy()

    def getCurrentCycle(self):
        return self.array

    def getCurrentType(self):
        return self.active_mode

    def run(self):
        pass

    def storeCurrent(self):
        self.last_active_mode = self.active_mode
        self.last_speed = self.speed
        self.last_brightness = self.brightness

    def setLED(self, type: config.Type, brightness: float = 1.0, speed: int = 5):
        self.storeCurrent()
        self.active_mode = type
        self.speed = speed
        self.brightness = brightness

    def getLED(self):
        if self.active_mode == None:
            return {
                'type': 0,
                'color': {
                    'r': 0,
                    'g': 0,
                    'b': 0
                }
            }
        else:
            return self.active_mode

    def setLast(self):
        self.active_mode = self.last_active_mode
        self.speed = self.last_speed
        self.brightness = self.last_brightness

    def match(self, type: config.Type):

        res = self.getArray()
        match type['type']:
            case 1:
                color = type['color']
                res = self._setStatic(color['r'], color['g'], color['b'])
            case 2:
                res = self._setRainbow()
            case 3:
                color = type['color']
                res = self._setTrack(color['r1'], color['g1'], color['b1'], color['r2'], color['g2'], color['b2'])
            case 4:
                color = type['color']
                res = self._setBlink(color['r'], color['g'], color['b'])
            case 5:
                percent = type['percent']
                res = self._setLadder(type['typeA'], type['typeB'], percent, type['speed'])
            case _:
                res = self._setRainbow()

        return res

    def cycle(self):
        '''
        cycles through LED array
        this should be called periodically
        '''
        self.array = self.match(self.active_mode)

        # self.m_led.setData(self.match(self.active_mode))

        self.m_led.setData(self.array)

    def _setStatic(self, red: int, green: int, blue: int):

        static = self.getArray()

        for i in range(self.size):
            static[i].setRGB(red, green, blue)
        # self.m_led.setData(self.array)

        return static

    def _setRainbow(self):
        arr = self.getArray()
        for i in range(self.size):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = math.floor(self.m_rainbowFirstPixelHue + (i * 180 / self.size) % 180)
            # Set the value
            arr[i].setHSV(hue, 255, 128)

        # Increase by to make the rainbow "move"
        self.m_rainbowFirstPixelHue += self.speed
        # Check bounds
        self.m_rainbowFirstPixelHue %= 180
        # self.m_led.setData(self.array)

        return arr.copy()

    def _setTrack(self, r1, g1, b1, r2, g2, b2):
        track = self.getArray()
        for i in range(self.size):
            track[i].setRGB(r1, g1, b1)

        for i in range(self.track_index, self.size, 4):
            track[i].setRGB(r2, g2, b2)

        self.track_index += 1

        if self.track_index > self.size:
            self.track_index = 0

        return track

        # self.m_led.setData(self.array)

    def _setBlink(self, r, g, b):
        blink = self.getArray()
        if self.blink_index / (2 * self.speed) <= .5:
            for i in range(self.size):
                blink[i].setRGB(r, g, b)
        else:
            for i in range(self.size):
                blink[i].setRGB(0, 0, 0)

        self.blink_index += 1
        if self.blink_index > 2 * self.speed:
            self.blink_index = 0

        return blink

    def _setLadder(self, typeA: config.Type, typeB: config.Type, percent: float, speed: int):

        if percent < 0:
            percent = 0
        elif percent > 1:
            percent = 1

        # print('a type', typeA['type'])

        # print('ladder percent:', percent)

        save = self.speed

        self.speed = speed

        b_led = self.match(typeB).copy()

        b = []

        for i, led_b in enumerate(b_led):
            if i < math.floor(self.size * percent):
                b.append(led_b)
        # print('b size:', len(b))

        # print('list b', len(b))

        self.speed = save

        a_led: list = self.match(typeA).copy()

        a: list = []

        for i, led_a in enumerate(a_led):
            if i < self.size - math.floor(self.size * percent):
                a.append(led_a)

        res = b + a

        if len(res) > self.size:
            del res[self.size:]
        else:
            return res.copy()

        return self.array


class SLEDS:
    """
    Switchable LEDS from Switchable PDH
    """

    def on(self):
        PowerDistribution.setSwitchableChannel(True)

    def off(self):
        PowerDistribution.setSwitchableChannel(False)
