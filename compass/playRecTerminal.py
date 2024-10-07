import threading
import subprocess
import time
import os
from scipy.io.wavfile import read
from datetime import datetime
# import RPi.GPIO as GPIO
#sys.path.insert(1,'/home/pi/compass/gy86/mpu.py')


def play(signal_path,signal,duration):
    cmd_play = ['aplay', '-Dplughw:0,0', '-c8', '-r96000', os.path.join(signal_path, signal)]
#    print(cmd_play)
    p = subprocess.run(cmd_play , timeout = duration)

def rec(records_path,timestr,duration,max_wav_duration):
    cmd_record = ['arecord', '-Dplughw:0,0', '-c8', '-r96000', os.path.join(records_path, timestr + '.wav'), '-d ' + str(duration)]
#    print(cmd_record)
    while duration > max_wav_duration:
        cmd_record[5] = '-d ' + str(max_wav_duration)
        timestr = datetime.utcnow().strftime("%Y%m%d-%H%M%S.%f")[:-3]
        cmd_record[4] = os.path.join(records_path, timestr + '.wav')
        duration -= max_wav_duration
        try:
            p = subprocess.run(cmd_record, timeout=max_wav_duration )
            time.sleep(max_wav_duration)
        except Exception as e:
            print('')
    try:
        cmd_record[5] = '-d ' + str(duration)
        timestr = datetime.utcnow().strftime("%Y%m%d-%H%M%S.%f")[:-3]
        cmd_record[4] = os.path.join(records_path, timestr + '.wav')
        p = subprocess.run(cmd_record, timeout=duration )
    except Exception as e:
        print('')

def compass():
    work_path = os.getcwd()
    p = subprocess.run(['python3',work_path + '/compass/magnet2.py'])


if __name__ == "__main__":
    # GPIO.setmode(GPIO.BCM) #set up mod to gpio numbers and not pin numbers
    # GPIO.setwarnings(False)
    # led_pin = 17
    # GPIO.setup(led_pin,GPIO.OUT)
    # GPIO.output(led_pin,GPIO.HIGH)
    os.chdir('/home/pi/raspbery_version')
    signal = 'chirp_30k-41k_15.wav'
#    signal = 'chirp_30k-41k_180.wav'
    work_path = os.getcwd()
    signal_path = work_path + '/signals/'
    records_path = work_path + '/records/'
    file = read(signal_path + signal)
    rate = file[0]
    frames = len(file[1])
    duration = int(frames / rate)
    timestr = datetime.utcnow().strftime("%Y%m%d-%H%M%S.%f")[:-3]
    max_wav_duration = 180
    try:
        # compass_thread = threading.Thread(target=compass,args=())
        # compass_thread.start()
        
        while True:
            compass_thread = threading.Thread(target=compass,args=())
            compass_thread.start()

            record_thread = threading.Thread(target=rec, args=(records_path, timestr, duration, max_wav_duration))
            play_thread = threading.Thread(target=play, args=(signal_path, signal, duration))
            
            #activate transmit and record
            record_thread.start()
            time.sleep(0.1)
            play_thread.start()

            #finish transmit and record
            play_thread.join()
            record_thread.join()
            os.system("/home/pi/raspbery_version/./restart-octo.sh ")
            
            compass_thread.join()
            time.sleep(5)
        # GPIO.output(led_pin,GPIO.LOW)
    except Exception as e:
        print(str(e))
        #GPIO.cleanup()

    
