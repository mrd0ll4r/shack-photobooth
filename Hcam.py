#!/usr/bin/python3

import os
import subprocess
import RPi.GPIO as GPIO
import threading
from collections import deque
from time import sleep
from subprocess import check_output
from datetime import datetime
import _thread
import picamera
from picamera import mmal
import ctypes as ct


class NoIRFixedPiCamera(picamera.PiCamera):
    """ Modified PiCamera with additional AWB mode to fix purple images on no-IR cameras.

    See https://github.com/raspberrypi/firmware/issues/1167#issuecomment-511798033
    See https://forums.raspberrypi.com/viewtopic.php?f=43&t=245994
    """
    AWB_MODES = {
        'off': mmal.MMAL_PARAM_AWBMODE_OFF,
        'auto': mmal.MMAL_PARAM_AWBMODE_AUTO,
        'sunlight': mmal.MMAL_PARAM_AWBMODE_SUNLIGHT,
        'cloudy': mmal.MMAL_PARAM_AWBMODE_CLOUDY,
        'shade': mmal.MMAL_PARAM_AWBMODE_SHADE,
        'tungsten': mmal.MMAL_PARAM_AWBMODE_TUNGSTEN,
        'fluorescent': mmal.MMAL_PARAM_AWBMODE_FLUORESCENT,
        'incandescent': mmal.MMAL_PARAM_AWBMODE_INCANDESCENT,
        'flash': mmal.MMAL_PARAM_AWBMODE_FLASH,
        'horizon': mmal.MMAL_PARAM_AWBMODE_HORIZON,
        'greyworld': ct.c_uint32(10)
    }


PRINT_ENABLED = True

""" Printer properties """
PRINTER_WIDTH = 640
PRINTER_HEIGHT = 384
PRINTER_SIZE = (PRINTER_WIDTH, PRINTER_HEIGHT)
PRINTER_SIZE_STR = "{}x{}".format(PRINTER_WIDTH, PRINTER_HEIGHT)

""" Camera image properties """
PHOTO_WIDTH = PRINTER_WIDTH * 4
PHOTO_HEIGHT = PRINTER_HEIGHT * 4
PHOTO_SIZE = (PHOTO_WIDTH, PHOTO_HEIGHT)

""" Camera video properties """
VIDEO_SIZE = (640, 480)
VIDEO_FRAMERATE = 10

"""
Pin numbers in standard BCM numbering.
LEDs are numbered in clockwise direction, starting with the leftmost one.
"""
PIN_PHOTO = 22
PIN_VIDEO = 23
PIN_FOOTSWITCH = 16
PIN_LED1 = 19
PIN_LED2 = 6
PIN_LED3 = 5
PIN_LED4 = 12
PIN_LED5 = 26
PIN_LED6 = 13
PIN_TUBES = 3
PIN_BULBS = 2
PINS_LED = [PIN_LED1, PIN_LED2, PIN_LED3, PIN_LED4, PIN_LED5, PIN_LED6]

""" Directory and file paths. """
FOLDER_RESULTS = "/home/pi/Pictures/"
FOLDER = "/home/pi/HCam/"
FOLDER_TEMP = "/home/pi/HCam/temp/"
FILE_WATERMARK_PRINT = FOLDER + "logo_U_print.png"
FILE_GROUPLINK_PRINT = FOLDER + "link_overlay_print.png"
FILE_COLORSCALE_PRINT = FOLDER + "colorscale.gif"
FILE_WATERMARK_BIG = FOLDER + "logo_big.png"
FILE_PRINT = FOLDER_TEMP + "print.png"
FILE_TEMP_BIG = FOLDER_TEMP + "temp.png"

""" A lock used to synchronize access to the status LEDs. """
LED_lock = _thread.allocate_lock()


def set_leds(to=None):
    LED_lock.acquire()
    set_leds_inner(to)
    LED_lock.release()


def set_leds_inner(to=None):
    assert len(to) == 6, "Invalid number of LEDs"

    GPIO.output(PIN_LED1, to[0])
    GPIO.output(PIN_LED2, to[1])
    GPIO.output(PIN_LED3, to[2])
    GPIO.output(PIN_LED4, to[3])
    GPIO.output(PIN_LED5, to[4])
    GPIO.output(PIN_LED6, to[5])


def indicate_standby():
    """ Displays the "standby" pattern, which is the leftmost LED active. """
    set_leds([1, 0, 0, 0, 0, 0])


def indicate_triangle_right():
    """ Displays a triangle pointing to the right. """
    set_leds([0, 1, 0, 1, 0, 1])


def indicate_triangle_left():
    """ Displays a triangle pointing to the left. """
    set_leds([1, 0, 1, 0, 1, 0])


class LoadingIndicator(threading.Thread):
    """ A thread that displays a loading animation on the LEDs.

    This fills the LEDs one by one in clockwise direction in total_time seconds.
    """

    def __init__(self, total_time=2.0):
        super(LoadingIndicator, self).__init__()
        self.total_time = total_time
        self.stop_request = threading.Event()

    def run(self):
        total_time = self.total_time
        sleep_time = total_time / 6.0
        led_pattern = [0, 0, 0, 0, 0, 0]

        LED_lock.acquire()

        for i in range(0, 6):
            led_pattern[i] = 1
            set_leds_inner(led_pattern)
            if self.stop_request.wait(sleep_time):
                # canceled
                LED_lock.release()
                return

        LED_lock.release()

    def join(self, timeout=None):
        super(LoadingIndicator, self).join(timeout)

    def stop(self):
        self.stop_request.set()


class WorkingIndicator(threading.Thread):
    """ A thread that displays a "busy" animation on the LEDs.

    This quickly loops through the LEDs one by one.
    """

    def __init__(self):
        super(WorkingIndicator, self).__init__()
        self.stop_request = threading.Event()

    def run(self):
        loop_time = .6
        sleep_time = loop_time / 6.0
        led_pattern = deque([1, 0, 0, 0, 0, 0])

        LED_lock.acquire()
        set_leds_inner(led_pattern)

        while not self.stop_request.wait(sleep_time):
            led_pattern.rotate(1)
            set_leds_inner(led_pattern)

        LED_lock.release()

    def join(self, timeout=None):
        super(WorkingIndicator, self).join(timeout)

    def stop(self):
        self.stop_request.set()


def print_image_file(filename):
    print("printing file " + filename)

    # Something somewhere hangs/is unfinished, probably some unterminated garbage at the end of a print via USB.
    # We flush that (maybe?) by writing to the USB Serial device.
    # Interestingly, that file is unavailable during printing, so we can use it to wait for the previous print to
    # finish.
    flushed = False
    while not flushed:
        try:
            with open('/dev/usb/lp0', 'w') as f:
                f.write('\r\n')
                flushed = True
        except (FileNotFoundError, PermissionError):
            print("waiting for previous print to finish (/dev/usb/lp0 unavailable or not writeable)")
            sleep(.5)

    subprocess.run(["lpr", filename])


def record_photo(filename):  # takes 4 photos at different lighting conditions
    print("recording " + filename)

    progress = LoadingIndicator(1.6)
    progress.start()

    camera.resolution = PHOTO_SIZE

    camera.start_preview()

    sleep(.5)  # pause to let the camera adjust
    camera.capture(filename + "_1.jpg")  # videoport might increase speed but implies a smaller viewing angle

    GPIO.output(PIN_TUBES, 1)  # turn on tubes

    sleep(.5)
    camera.capture(filename + "_2.jpg")

    GPIO.output(PIN_BULBS, 1)  # turn on bulbs

    sleep(1)  # pause to let the camera adjust and heat up the bulbs
    camera.capture(filename + "_3.jpg")

    GPIO.output(PIN_TUBES, 0)  # turn off tubes

    sleep(.5)
    camera.capture(filename + "_4.jpg")

    GPIO.output(PIN_BULBS, 0)  # turn off bulbs

    camera.stop_preview()

    print("finished " + filename)
    progress.stop()


def record_video(filename):
    progress = LoadingIndicator(2)
    progress.start()

    camera.resolution = VIDEO_SIZE


def setup_gpio():
    # Set numbering to BCM.
    GPIO.setmode(GPIO.BCM)

    # Inputs
    GPIO.setup(PIN_PHOTO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_VIDEO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_FOOTSWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Outputs
    GPIO.setup(PIN_TUBES, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_BULBS, GPIO.OUT, initial=GPIO.LOW)
    for p in PINS_LED:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

    # Set up detection of button events.
    GPIO.add_event_detect(PIN_PHOTO, GPIO.FALLING, bouncetime=10000)  # this will block for 30s
    GPIO.add_event_detect(PIN_FOOTSWITCH, GPIO.FALLING, bouncetime=1000)
    GPIO.add_event_detect(PIN_VIDEO, GPIO.FALLING, bouncetime=1000)


def setup_camera():
    camera = NoIRFixedPiCamera()
    camera.rotation = 0
    camera.framerate = VIDEO_FRAMERATE
    camera.contrast = 60
    camera.exposure_mode = 'auto'
    # This fixes an issue with recent firmwares and NoIR cameras.
    # See
    camera.awb_mode = 'greyworld'
    return camera


def setup_colorscale():
    subprocess.run(["convert", "xc:gray10", "xc:gray90", "+append", FILE_COLORSCALE_PRINT])


def prepare_print_file(print_input_file):
    subprocess.run(
        ["convert", print_input_file, "-resize", PRINTER_SIZE_STR, "-auto-gamma", "-brightness-contrast",
         "20x10", FILE_PRINT])
    subprocess.run(["convert", "-composite", FILE_PRINT, FILE_WATERMARK_PRINT, FILE_PRINT])
    subprocess.run(["convert", FILE_PRINT, "-remap", FILE_COLORSCALE_PRINT, FILE_PRINT])
    subprocess.run(["convert", "-composite", FILE_PRINT, FILE_GROUPLINK_PRINT, FILE_PRINT])


setup_gpio()

# indicate startup process
indicator = WorkingIndicator()
indicator.start()

# get IP adress
hostIP = check_output(['hostname', '-I'])
print(hostIP)

# Create camera and in-memory stream
camera = setup_camera()

setup_colorscale()

indicator.stop()

indicate_standby()  # waits for the LED lock, so effectively for progress to actually stop

try:

    print("entering main loop")
    # Main loop
    while True:

        sleep(.2)

        if GPIO.event_detected(PIN_FOOTSWITCH):
            # we just want to test the footswitch...
            print("footswitch pressed")
            indicate_triangle_right()
            sleep(.5)
            indicate_standby()

        if GPIO.event_detected(PIN_VIDEO):
            # we just want to test the video button...
            print("video pressed")
            indicate_triangle_left()
            sleep(.5)
            indicate_standby()

        # take a picture   
        if GPIO.event_detected(PIN_PHOTO):
            now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            photo_basename = FOLDER_TEMP + now

            photo_for_result = photo_basename + "_1.jpg"
            photo_for_result_flash = photo_basename + "_4.jpg"
            result_filename = FOLDER_RESULTS + now + ".png"
            result_filename_flash = FOLDER_RESULTS + now + "_flash.png"
            record_photo(photo_basename)  # produce 4 numbered fotos on disk. This has its own progress indicator.

            indicator = WorkingIndicator()
            indicator.start()

            os.chdir(FOLDER_TEMP)

            if PRINT_ENABLED:
                # We print the first image because it looks best at night (as determined through trial and error).
                # During daytime images 3 and 4 look better, but we don't optimize for that case :)

                print_input_file = "{}_1.jpg".format(now)
                prepare_print_file(print_input_file)
                print_image_file(FILE_PRINT)

            # postprocess the high quality saved image (takes long)
            print("postprocessing saved image file...")

            subprocess.run(["convert", photo_for_result, "-auto-level", "-auto-gamma", "-unsharp", "2", FILE_TEMP_BIG])
            subprocess.run(["convert", "-composite", FILE_TEMP_BIG, FILE_WATERMARK_BIG, result_filename])

            subprocess.run(
                ["convert", photo_for_result_flash, "-auto-level", "-auto-gamma", "-unsharp", "2", FILE_TEMP_BIG])
            subprocess.run(["convert", "-composite", FILE_TEMP_BIG, FILE_WATERMARK_BIG, result_filename_flash])

            # make sure the result is readable even if the program runs as a different user
            # os.system("chown pi:users " + result_filename)
            # os.system("chown pi:users " + result_filename_flash)

            print("done processing image")
            indicator.stop()
            indicate_standby()


except KeyboardInterrupt:
    print("Main loop has exited")
    GPIO.cleanup()
