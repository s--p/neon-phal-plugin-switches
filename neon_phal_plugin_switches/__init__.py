import RPi.GPIO as GPIO
from ovos_plugin_manager.phal import PHALPlugin
from ovos_plugin_manager.hardware.switches import AbstractSwitches
from ovos_utils.log import LOG
from ovos_bus_client.message import Message

class SwitchInputs(PHALPlugin):
    def __init__(self, bus=None, config=None):
        super().__init__(bus=bus, name="neon-phal-plugin-switches", config=config)
        self.switches = GPIOSwitches(action_callback=self.on_button_press)
        self.switches.on_action = self.on_button_press

    def on_button_press(self):
        LOG.info("Listen button pressed")
        self.bus.emit(Message("mycroft.mic.listen"))

class GPIOSwitches(AbstractSwitches):
    def __init__(self, action_callback, action_pin: int = 17):
        self.on_action = action_callback
        self.action_pin = action_pin
        self.setup_gpio()

    def setup_gpio(self, debounce=100):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.action_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.action_pin,
                              GPIO.FALLING,
                              callback=self.handle_action,
                              bouncetime=debounce)

    def handle_action(self, _):
        self.on_action()

    @property
    def capabilities(self) -> dict:
        return {}

    def shutdown(self):
        GPIO.cleanup()
