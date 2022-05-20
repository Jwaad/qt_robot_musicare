#!/usr/bin/env python

import os
from pydub import AudioSegment

input_path = "input/"
output_path = "output/"

input_contents = os.listdir(input_path)

for file_name in input_contents:
    file_path= os.path.join(input_path, file_name)
    save_name = file_name[:-4]+".wav"
    save_path = os.path.join(output_path, save_name)
    print(file_path, save_path)
    
    sound = AudioSegment.from_mp3(file_path)
    sound.export(save_path, format="wav")
    
