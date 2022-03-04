#!/usr/bin/python3

'''
Version 4, 2022/03/02
@author: Leo Balduf

Version 3, 25 May 2019
@author: Paul Balduf

following @autor muth, 24 Jan 2017   
'''
import os
import RPi.GPIO as GPIO
import threading
import time
from time import sleep
from PIL import Image
from PIL import ImageOps
from PIL import ImageEnhance
from PIL import ImageDraw
from PIL import ImageFont
from picamera import PiCamera
from io import BytesIO
from subprocess import check_output
from datetime import datetime
import _thread


PRINT_ENABLED=1


PRINTER_WIDTH = 640
PRINTER_HEIGHT = 384
PRINTER_SIZE = (PRINTER_WIDTH, PRINTER_HEIGHT)

PHOTO_WIDTH = PRINTER_WIDTH*4
PHOTO_HEIGHT = PRINTER_HEIGHT*4
PHOTO_SIZE = (PHOTO_WIDTH, PHOTO_HEIGHT)

VIDEO_SIZE = (640, 480)
VIDEO_FRAMERATE=10


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


FOLDER_RESULTS = "/home/pi/Pictures/"
FOLDER = "/home/pi/HCam/"
FOLDER_TEMP = "/home/pi/HCam/temp/"


LED_lock = _thread.allocate_lock()



def indicate_standby():
    LED_lock.acquire()
    GPIO.output(PIN_LED1, 1)
    GPIO.output(PIN_LED2, 0)
    GPIO.output(PIN_LED3, 0)
    GPIO.output(PIN_LED4, 0)
    GPIO.output(PIN_LED5, 0)
    GPIO.output(PIN_LED6, 0)
    LED_lock.release()

# displays a triangle pointing to the right for debugging purposes  
def indicate_triangle_right():
    LED_lock.acquire()
    GPIO.output(PIN_LED1, 0)
    GPIO.output(PIN_LED2, 1)
    GPIO.output(PIN_LED3, 0)
    GPIO.output(PIN_LED4, 1)
    GPIO.output(PIN_LED5, 0)
    GPIO.output(PIN_LED6, 1)
    LED_lock.release()

def indicate_triangle_left():
    LED_lock.acquire()
    GPIO.output(PIN_LED1, 1)
    GPIO.output(PIN_LED2, 0)
    GPIO.output(PIN_LED3, 1)
    GPIO.output(PIN_LED4, 0)
    GPIO.output(PIN_LED5, 1)
    GPIO.output(PIN_LED6, 0)
    LED_lock.release()



class Progress_indicator(threading.Thread): 
    def __init__(self, total_time):
        super(Progress_indicator, self).__init__()
        self.total_time = total_time
        self.stoprequest = threading.Event()

    def run(self):
        
        total_time=self.total_time
        LED_lock.acquire()
        
        GPIO.output(PIN_LED1, 0)
        GPIO.output(PIN_LED2, 0)
        GPIO.output(PIN_LED3, 0)
        GPIO.output(PIN_LED4, 0)
        GPIO.output(PIN_LED5, 0)
        GPIO.output(PIN_LED6, 0)
    
        sleep(total_time/6.0)
        GPIO.output(PIN_LED1, 1)
    
        if self.stoprequest.isSet(): 
            LED_lock.release()
            return
        sleep(total_time/6.0)
        GPIO.output(PIN_LED2, 1)

        if self.stoprequest.isSet(): 
            LED_lock.release()
            return
        sleep(total_time/6.0)
        GPIO.output(PIN_LED3, 1)

        if self.stoprequest.isSet(): 
            LED_lock.release()
            return
        sleep(total_time/6.0)
        GPIO.output(PIN_LED4, 1)
    
        if self.stoprequest.isSet(): 
            LED_lock.release()
            return
        sleep(total_time/6.0)
        GPIO.output(PIN_LED5, 1)

        if self.stoprequest.isSet(): 
            LED_lock.release()
            return
        sleep(total_time/6.0)
        GPIO.output(PIN_LED6, 1)
        
        LED_lock.release()

    def join(self, timeout=None):
        super(Progress_indicator, self).join(timeout)
        
    def stop(self):
        self.stoprequest.set()

   
class Working_indicator(threading.Thread): 
    def __init__(self):
        super(Working_indicator, self).__init__()
        self.stoprequest = threading.Event()

    def run(self):
        
        
        indicate_standby() 
        loop_time=.6
    
        LED_lock.acquire()
    
        while not self.stoprequest.isSet() :
            sleep(loop_time/6.0)
            GPIO.output(PIN_LED1, 0)
            GPIO.output(PIN_LED2, 1)
    
            sleep(loop_time/6.0)
            GPIO.output(PIN_LED2, 0)
            GPIO.output(PIN_LED3, 1)
    
            sleep(loop_time/6.0)
            GPIO.output(PIN_LED3, 0)
            GPIO.output(PIN_LED4, 1)
    
            sleep(loop_time/6.0)
            GPIO.output(PIN_LED4, 0)
            GPIO.output(PIN_LED5, 1)
    
            sleep(loop_time/6.0)
            GPIO.output(PIN_LED5, 0)
            GPIO.output(PIN_LED6, 1)
    
            sleep(loop_time/6.0)
            GPIO.output(PIN_LED6, 0)
            GPIO.output(PIN_LED1, 1)
        
        LED_lock.release()

    def join(self, timeout=None):
        super(Working_indicator, self).join(timeout)
        
    def stop(self):
        self.stoprequest.set()
    


    
def print_image_file(filename):
    print("printing file "+ filename)

    # Something somewhere hangs/is unfinished, probably some unterminated garbage at the end of a print via USB.
    # We flush that (maybe?) by writing to the USB Serial device.
    with open('/dev/usb/lp0', 'w') as f:
        f.write('\r\n')
    
    #os.system("lpr -o fit-to-page "+filename)
    os.system("lpr "+filename)
    
    ''' 
    try:
        image = Image.open(filename)
        im_width, im_height = image.size
        if im_width > im_height:
            image = image.rotate(90, expand=1)
            im_width, im_height = image.size
        ratio = (PRINTER_HEIGHT/float(im_width))
        height = int((float(im_height)*float(ratio)))
        image = image.resize((PRINTER_HEIGHT, height), Image.ANTIALIAS)
        
        printer.printImage(image, False)
        
        printer.feed(3)
    except IOError:
        print ("cannot identify image file", filename)
    '''  

def record_photo(filename): #takes 4 photos at different lighting conditions
    
    progress = Progress_indicator(1.6)
    progress.start()
    
    print("recording "+filename)
    
    camera.resolution = PHOTO_SIZE
    
    camera.start_preview()
    
    sleep(.5)   # pause to let the camera adjust
    camera.capture(filename+"_1.jpg", use_video_port=False) #videoport might increase speed but implies a smaller viewing angle           
    
    GPIO.output(PIN_TUBES, 1)   # turn on tubes 
                
    sleep(.5)
    camera.capture(filename+"_2.jpg", use_video_port=False) 
    
    GPIO.output(PIN_BULBS, 1)   # turn on bulbs
    
    sleep(1)    # pause to let the camera adjust and heat up the bulbs
    camera.capture(filename+"_3.jpg", use_video_port=False) 
    
    GPIO.output(PIN_TUBES, 0)   # turn off tubes
    
    sleep(.5)
    camera.capture(filename+"_4.jpg", use_video_port=False) 
    
    GPIO.output(PIN_BULBS, 0)   # turn off bulbs
    
    
    camera.stop_preview()

    print("finished "+filename)
    progress.stop()


def record_video(filename):
    
    progress = Progress_indicator(2)
    progress.start()
    
    camera.resolution = VIDEO_SIZE
    



# GPIO setup. Standard pin numbering (i.e. GPIO#, not pin number on the board layout)
GPIO.setmode(GPIO.BCM)

GPIO.setup(PIN_PHOTO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_VIDEO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_FOOTSWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_LED1, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(PIN_LED2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_LED3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_LED4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_LED5, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_LED6, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_TUBES, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_BULBS, GPIO.OUT, initial=GPIO.LOW)


# indicate startup process
progress = Working_indicator()
progress.start()




# get IP adress
hostIP = check_output(['hostname', '-I'])
print (hostIP)



# setup detection of button events
GPIO.add_event_detect(PIN_PHOTO, GPIO.FALLING, bouncetime=10000)  #this will block for 30s
GPIO.add_event_detect(PIN_FOOTSWITCH, GPIO.FALLING, bouncetime=1000)  
GPIO.add_event_detect(PIN_VIDEO, GPIO.FALLING, bouncetime=1000)  


# Create camera and in-memory stream
stream = BytesIO()
camera = PiCamera()
camera.rotation = 0
camera.framerate = VIDEO_FRAMERATE
camera.contrast = 60
camera.exposure_mode = 'auto'
camera.awb_mode= 'flash'




    
progress.stop()

indicate_standby()  # waits for the LED lock, so effectively for progress to actually stop

try:

    print("entering main loop")
    #Main loop 
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
            filename=datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            record_photo(FOLDER + "temp/" + filename) # produce 4 numbered fotos on disk. This has its own progress indicator.
                
            progress = Working_indicator()
            progress.start()
            
            

            os.chdir(FOLDER_TEMP)


                       
            if(PRINT_ENABLED):
                #prepare the file for print
                os.system("convert "+filename+"_1.jpg -resize " + str(PRINTER_WIDTH) + "x" + str(PRINTER_HEIGHT) + " -auto-gamma print.png")
                os.system("convert xc:gray10 xc:gray90 +append colorscale.gif")
                os.system("convert -composite print.png "+ FOLDER + "logo_U_print.png print.png")
                #os.system("convert -composite print.png "+ FOLDER + "qr-overlay.png print.png")
                os.system("convert print.png -remap colorscale.gif print.png")
                os.system("convert -composite print.png "+ FOLDER + "qr-overlay.png print.png")

                print_image_file("print.png")
                #os.system("rm print.png")
               
            #postprocess the high quality saved image (takes long)
            print("postprocessing saved image file...")
                
            os.system("convert " +  filename + "_1.jpg -auto-level -auto-gamma -unsharp 2 temp.png")
            os.system("convert -composite temp.png " + FOLDER + "logo_big.png " + FOLDER_RESULTS + filename + ".png")
        
            #make sure the result is readable even if the program runs as a different user
            os.system("chown pi:users "+FOLDER_RESULTS + filename + ".png")
                
            os.system("rm temp.png")
                 
            print("done processing image")
            progress.stop()
            indicate_standby()
    
        '''
            
        if GPIO.event_detected(PIN_VIDEO):
           
            filename=datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                
            record_video(filename)
            
            indicate_standby()
        '''    
            
           
    
except KeyboardInterrupt:        
    print("Main loop has exited")
    GPIO.cleanup()


