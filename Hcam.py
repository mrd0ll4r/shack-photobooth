#!/usr/bin/python3

import os
import subprocess
import RPi.GPIO as GPIO
import threading
from collections import deque
from time import sleep
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
REMOVE_TEMP_FILES = True

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
DIR_RESULTS = "/home/pi/Pictures/"
DIR_TMP = "./tmp/"

# Colorscale for printing. This is generated on the fly.
FILE_COLORSCALE_PRINT = "colorscale.gif"

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


def print_image_file(filename: str):
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

    subprocess.run(["lpr", filename], check=True)


class RecordedPhotos:
    timestamp = None
    no_lights_path = None
    tubes_path = None
    tubes_bulbs_path = None
    bulbs_path = None

    @staticmethod
    def tmp_jpeg_with_suffix(timestamp: str, suffix: str):
        return os.path.join(DIR_TMP, "{}_{}.jpg".format(timestamp, suffix))

    def __init__(self, timestamp: str):
        self.timestamp = timestamp
        self.no_lights_path = self.tmp_jpeg_with_suffix(timestamp, "no_lights")
        self.tubes_path = self.tmp_jpeg_with_suffix(timestamp, "tubes")
        self.tubes_bulbs_path = self.tmp_jpeg_with_suffix(timestamp, "tubes_bulbs")
        self.bulbs_path = self.tmp_jpeg_with_suffix(timestamp, "bulbs")

    def __del__(self):
        if REMOVE_TEMP_FILES:
            os.remove(self.no_lights_path)
            os.remove(self.tubes_path)
            os.remove(self.tubes_bulbs_path)
            os.remove(self.bulbs_path)


class OutputImages:
    FILE_TEMP_BIG = os.path.join(DIR_TMP, "temp_big.png")
    FILE_BIG_WATERMARK = "logo_big.png"

    timestamp = None
    no_lights_path = None
    flash_path = None
    social_media_path = None
    recorded_photos = None

    @staticmethod
    def result_filename(timestamp: str, suffix: str = None):
        if suffix is not None:
            return os.path.join(DIR_RESULTS, "{}_{}.jpg".format(timestamp, suffix))
        return os.path.join(DIR_RESULTS, "{}.jpg".format(timestamp))

    def __init__(self, recorded_photos: RecordedPhotos):
        self.timestamp = recorded_photos.timestamp
        self.recorded_photos = recorded_photos
        self.no_lights_path = self.result_filename(self.timestamp)
        self.flash_path = self.result_filename(self.timestamp, "flash")
        self.social_media_path = self.result_filename(self.timestamp, "social_media")

    @staticmethod
    def composite_with_watermark(input_path: str, output_path: str):
        subprocess.run(["convert", "-composite", input_path, OutputImages.FILE_BIG_WATERMARK, output_path], check=True)

    def process(self):
        subprocess.run(
            ["convert", self.recorded_photos.no_lights_path, "-auto-level", "-auto-gamma", "-unsharp", "2",
             OutputImages.FILE_TEMP_BIG], check=True)
        self.composite_with_watermark(OutputImages.FILE_TEMP_BIG, self.no_lights_path)

        subprocess.run(
            ["convert", self.recorded_photos.bulbs_path, "-auto-level", "-auto-gamma", "-unsharp", "2",
             OutputImages.FILE_TEMP_BIG],
            check=True)
        # Alternatively
        # subprocess.run(
        #    ["convert", self.recorded_photos.bulbs_path, "-normalize", "-adaptive-sharpen", "0x3", "-modulate", "100,70",
        #     OutputImages.FILE_TEMP_BIG],check=True)
        self.composite_with_watermark(OutputImages.FILE_TEMP_BIG, self.flash_path)

        subprocess.run(
            ["convert", self.recorded_photos.tubes_bulbs_path, "-normalize", "-adaptive-sharpen", "0x10", "-modulate",
             "100,130",
             OutputImages.FILE_TEMP_BIG], check=True)
        self.composite_with_watermark(OutputImages.FILE_TEMP_BIG, self.social_media_path)


def record_photos(camera: picamera.PiCamera, timestamp: str) -> RecordedPhotos:
    """
    Takes four photos at different lighting conditions.
    Parameters are paths to jpeg files.
    """

    photo_paths = RecordedPhotos(timestamp)

    # We're using camera.capture.
    # videoport might increase speed but implies a smaller viewing angle

    print("recording photos")
    progress = LoadingIndicator(1.6)
    progress.start()

    camera.resolution = PHOTO_SIZE

    camera.start_preview()

    # pause to let the camera adjust
    sleep(.5)
    camera.capture(photo_paths.no_lights_path)

    # turn on tubes
    GPIO.output(PIN_TUBES, 1)

    sleep(.5)
    camera.capture(photo_paths.tubes_path)

    # turn on bulbs
    GPIO.output(PIN_BULBS, 1)

    # longer pause to let the camera adjust and heat up the bulbs
    sleep(1)
    camera.capture(photo_paths.tubes_bulbs_path)

    # turn off tubes
    GPIO.output(PIN_TUBES, 0)

    sleep(.5)
    camera.capture(photo_paths.bulbs_path)

    # turn off bulbs
    GPIO.output(PIN_BULBS, 0)

    camera.stop_preview()

    print("finished recording photos")
    progress.stop()

    return photo_paths


def record_video(camera, filename):
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


def setup_camera() -> NoIRFixedPiCamera:
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
    subprocess.run(["convert", "xc:gray10", "xc:gray90", "+append", FILE_COLORSCALE_PRINT], check=True)


def prepare_print_file_inner(input_image_path: str, output_image_path: str, pre_grayscale_composite_paths: [str],
                             post_grayscale_composite_paths: [str]):
    subprocess.run(
        ["convert", input_image_path, "-resize", PRINTER_SIZE_STR, "-auto-gamma", "-brightness-contrast",
         "20x10", output_image_path], check=True)
    for watermark in pre_grayscale_composite_paths:
        subprocess.run(["convert", "-composite", output_image_path, watermark, output_image_path], check=True)

    subprocess.run(["convert", output_image_path, "-remap", FILE_COLORSCALE_PRINT, output_image_path], check=True)

    for watermark in post_grayscale_composite_paths:
        subprocess.run(["convert", "-composite", output_image_path, watermark, output_image_path], check=True)

    # TODO add some text below, like this:
    # Create text image:
    # convert -font DejaVu-Sans-Mono -pointsize 38 -size 374x182 label:"$(date '+%d.%m.%Y %H:%M')" -bordercolor white -border 5 timestamp.png
    # Make print larger (wider):
    # convert print.png -background white -gravity west -extent 832x384 print-extended.png
    # Copy the timestamp into the extended print image:
    # convert print-extended.png timestamp.png -gravity east -composite print-final.png


def prepare_print_file(recorded_photos: RecordedPhotos) -> str:
    print_file_path = os.path.join(DIR_TMP, "print.png")
    file_watermark_print = "logo_U_print.png"
    file_group_link_print = "link_overlay_print.png"

    # We print the first image (without lighting) because it looks best at night (as determined through trial and
    # error). During daytime the images with bulbs look better, but we don't optimize for that case :)
    photo_for_print = recorded_photos.no_lights_path

    prepare_print_file_inner(photo_for_print, print_file_path, [file_watermark_print], [file_group_link_print])
    # TODO print date, maybe a funny quote.
    # Maybe tommy cash lyrics?

    return print_file_path


def process_output_files(recorded_photos: RecordedPhotos) -> OutputImages:
    output_images = OutputImages(recorded_photos)

    print("processing output images")
    output_images.process()

    return output_images


def main():
    setup_gpio()

    # indicate startup process
    indicator = WorkingIndicator()
    indicator.start()

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

            if GPIO.event_detected(PIN_PHOTO):
                # take a picture
                ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

                # Record photos. This has its own progress indicator.
                recorded_photos = record_photos(camera, ts)

                indicator = WorkingIndicator()
                indicator.start()

                if PRINT_ENABLED:
                    file_to_print = prepare_print_file(recorded_photos)

                    print_image_file(file_to_print)

                # postprocess the high quality saved image (takes long)
                print("postprocessing saved image file...")

                process_output_files(recorded_photos)

                print("done processing image")

                indicator.stop()
                indicate_standby()

    except KeyboardInterrupt:
        print("Main loop exiting")
        GPIO.cleanup()


if __name__ == "__main__":
    main()
