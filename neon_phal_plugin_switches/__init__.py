import RPi.GPIO as GPIO
from abc import ABC
from time import sleep
from ovos_plugin_manager.phal import PHALPlugin
from ovos_plugin_manager.hardware.switches import AbstractSwitches
from ovos_utils.log import LOG
from ovos_bus_client.message import Message

class SwitchInputs(PHALPlugin):
    def __init__(self, bus=None, config=None):
        super().__init__(bus=bus, name="neon-phal-plugin-switches", config=config)
        # TODO: Read pins from configuration
        self.switches = GPIOSwitches(action_callback=self.on_button_press,
                                     volup_callback=self.on_button_volup_press,
                                     voldown_callback=self.on_button_voldown_press,
                                     mute_callback=self.on_hardware_mute,
                                     unmute_callback=self.on_hardware_unmute)
        self.switches.on_mute = self.on_hardware_mute
        self.switches.on_unmute = self.on_hardware_unmute
        self.switches.on_action = self.on_button_press
        self.switches.on_vol_up = self.on_button_volup_press
        self.switches.on_vol_down = self.on_button_voldown_press

        if GPIO.input(self.switches.mute_pin) == self.switches.muted:
            self.bus.emit(Message('mycroft.mic.mute'))

        self.bus.on('mycroft.mic.status', self.on_mic_status)

    def on_mic_status(self, message):
        try:
            if GPIO.input(self.switches.mute_pin) == self.switches.muted:
                msg_type = 'mycroft.mic.mute'
            else:
                msg_type = 'mycroft.mic.unmute'
            self.bus.emit(message.reply(msg_type))
        except Exception as e:
            LOG.exception("Error in on_mic_status: {}".format(e))

    def on_button_press(self):
        try:
            LOG.info("Listen button pressed")
            if GPIO.input(self.switches.mute_pin) != self.switches.muted:
                self.bus.emit(Message("mycroft.mic.listen"))
            else:
                self.bus.emit(Message("mycroft.mic.error", {"error": "mic_sw_muted"}))
        except Exception as e:
            LOG.exception("Error in on_button_press: {}".format(e))

    def on_button_volup_press(self):
        try:
            LOG.debug("VolumeUp button pressed")
            self.bus.emit(Message("mycroft.volume.increase"))
        except Exception as e:
            LOG.exception("Error in on_button_volup_press: {}".format(e))

    def on_button_voldown_press(self):
        try:
            LOG.debug("VolumeDown button pressed")
            self.bus.emit(Message("mycroft.volume.decrease"))
        except Exception as e:
            LOG.exception("Error in on_button_voldown_press: {}".format(e))

    def on_hardware_mute(self):
        try:
            LOG.debug("mic HW muted")
            self.bus.emit(Message("mycroft.mic.mute"))
        except Exception as e:
            LOG.exception("Error in on_hardware_mute: {}".format(e))

    def on_hardware_unmute(self):
        try:
            LOG.debug("mic HW unmuted")
            self.bus.emit(Message("mycroft.mic.unmute"))
        except Exception as e:
            LOG.exception("Error in on_hardware_unmute: {}".format(e))


class GPIOSwitches(AbstractSwitches, ABC):
    def __init__(self, action_callback, volup_callback, voldown_callback,
                 mute_callback, unmute_callback, volup_pin: int = 22,
                 voldown_pin: int = 23, action_pin: int = 17,
                 mute_pin: int = 25, sw_active_state: int = 0,
                 sw_muted_state: int = 1):
        self.on_action = action_callback
        self.on_vol_up = volup_callback
        self.on_vol_down = voldown_callback
        self.on_mute = mute_callback
        self.on_unmute = unmute_callback

        self.vol_up_pin = volup_pin
        self.vol_dn_pin = voldown_pin
        self.action_pin = action_pin
        self.mute_pin = mute_pin
        self._active = sw_active_state
        self._muted = sw_muted_state

        self.setup_gpio()

    @property
    def muted(self):
        """
        Returns the state associated with 'mute' (0 for low, 1 for hi)
        """
        return self._muted

    def setup_gpio(self, debounce=100):
        """
        Do GPIO setup.
        """
        try:
            # use BCM GPIO pin numbering
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            if self._active == 0:
                pull_up_down = GPIO.PUD_UP
            else:
                pull_up_down = GPIO.PUD_DOWN
            # we need to pull up the 3 buttons and mute switch
            GPIO.setup(self.action_pin, GPIO.IN, pull_up_down=pull_up_down)
            GPIO.setup(self.vol_up_pin, GPIO.IN, pull_up_down=pull_up_down)
            GPIO.setup(self.vol_dn_pin, GPIO.IN, pull_up_down=pull_up_down)
            GPIO.setup(self.mute_pin, GPIO.IN, pull_up_down=pull_up_down)

            # attach callbacks
            GPIO.add_event_detect(self.action_pin,
                                  GPIO.BOTH,
                                  callback=self.handle_action,
                                  bouncetime=debounce)

            GPIO.add_event_detect(self.vol_up_pin,
                                  GPIO.BOTH,
                                  callback=self.handle_vol_up,
                                  bouncetime=debounce)

            GPIO.add_event_detect(self.vol_dn_pin,
                                  GPIO.BOTH,
                                  callback=self.handle_vol_down,
                                  bouncetime=debounce)

            GPIO.add_event_detect(self.mute_pin,
                                  GPIO.BOTH,
                                  callback=self.handle_mute,
                                  bouncetime=debounce)
        except Exception as e:
            LOG.exception("Error in setup_gpio: {}".format(e))

    def handle_action(self, _):
        try:
            if GPIO.input(self.action_pin) == self._active:
                self.on_action()
        except Exception as e:
            LOG.exception("Error in handle_action: {}".format(e))

    def handle_vol_up(self, _):
        try:
            if GPIO.input(self.vol_up_pin) == self._active:
                self.on_vol_up()
        except Exception as e:
            LOG.exception("Error in handle_vol_up: {}".format(e))

    def handle_vol_down(self, _):
        try:
            if GPIO.input(self.vol_dn_pin) == self._active:
                self.on_vol_down()
        except Exception as e:
            LOG.exception("Error in handle_vol_down: {}".format(e))

    def handle_mute(self, _):
        try:
            sleep(0.05)
            if GPIO.input(self.mute_pin) == self._muted:
                self.on_mute()
            else:
                self.on_unmute()
        except Exception as e:
            LOG.exception("Error in handle_mute: {}".format(e))

    @property
    def capabilities(self) -> dict:
        return {}

    def shutdown(self):
        try:
            # clean up GPIO on shutdown
            GPIO.cleanup()
        except Exception as e:
            LOG.exception("Error in shutdown: {}".format(e))
