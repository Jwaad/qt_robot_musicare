#!/usr/bin/env python

import os
import sys
import contextlib
import wave
from pydub import AudioSegment

wrongly_used = False

if len(sys.argv) != 3:
    print("usage: my_node.py arg1 arg2")
    wrongly_used = True
else:
    start = int(sys.argv[1]) #to be changed by parameters
    end = int(sys.argv[2]) #to be changed by parameters

if not wrongly_used:
    input_path = "input/"
    output_path = "output/"

    input_contents = os.listdir(input_path)

    for file_name in input_contents:
        file_path= os.path.join(input_path, file_name)
        
        #convert sec to milli sec
        start_len = (start) * 1000
        end_len = (end) * 1000

        #save new slice
        sound = AudioSegment.from_wav(file_path)
        sound = sound[start_len : end_len]
        
        save_path = os.path.join(output_path, file_name)
        
        sound.export(save_path, format="wav")
        print("Saved to:", save_path)

