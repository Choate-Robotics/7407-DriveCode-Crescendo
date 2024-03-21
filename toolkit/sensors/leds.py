from wpilib import AddressableLED
import math, config


class ALeds:
    m_led: AddressableLED

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
        self.m_led.setLength(self.size) # 27
        self.m_ledBuffer = self.m_led.LEDData()
        self.led_data = [self.m_ledBuffer for i in range(self.size)]
        self.m_led.setData(self.led_data)

    def enable(self):
        self.m_led.start()

    def disable(self):
        self.m_led.stop()

    def set_brightness(self, brightness: float):
        self.brightness = brightness

    def get_led_data(self):
        return [self.m_led.LEDData() for i in range(self.size)].copy()

    def get_current_cycle(self):
        return self.led_data

    def get_current_type(self):
        if self.active_mode is None:
            return {
                'type': 0,
                'color': {
                    'r': 0,
                    'g': 0,
                    'b': 0
                }
            }
        return self.active_mode

    def store_current(self):
        self.last_active_mode = self.active_mode
        self.last_speed = self.speed
        self.last_brightness = self.brightness

    def reset_LED(self, type: config.Type, brightness: float = 1.0, speed: int = 5):
        self.store_current()
        self.active_mode = type
        self.speed = speed
        self.brightness = brightness

    def set_last_current(self):
        self.active_mode = self.last_active_mode
        self.speed = self.last_speed
        self.brightness = self.last_brightness

    def match(self, type: config.Type):
        res = self.get_led_data()
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
        self.m_led.setData(self.match(self.active_mode))

    def _setStatic(self, red: int, green: int, blue: int):

        static = self.get_led_data()

        for i in range(self.size):
            static[i].setRGB(red, green, blue)

        return static

    def _setRainbow(self):
        rainbow = self.get_led_data()
        for i in range(self.size):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = math.floor(self.m_rainbowFirstPixelHue + (i * 180 / self.size) % 180)
            # Set the value
            rainbow[i].setHSV(hue, 255, 128)

        # Increase by to make the rainbow "move"
        self.m_rainbowFirstPixelHue += self.speed

        # Check bounds
        self.m_rainbowFirstPixelHue %= 180

        return rainbow

    def _setTrack(self, r1, g1, b1, r2, g2, b2):
        track = self.get_led_data()
        for i in range(self.size):
            track[i].setRGB(r1, g1, b1)

        for i in range(self.track_index, self.size, 4):
            track[i].setRGB(r2, g2, b2)

        self.track_index += 1

        if self.track_index > self.size:
            self.track_index = 0

        return track

    def _setBlink(self, r, g, b):
        blink = self.get_led_data()
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

        save = self.speed

        self.speed = speed

        b_led = self.match(typeB).copy()

        b = []

        for i, led_b in enumerate(b_led):
            if i < math.floor(self.size * percent):
                b.append(led_b)

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

        return self.led_data
