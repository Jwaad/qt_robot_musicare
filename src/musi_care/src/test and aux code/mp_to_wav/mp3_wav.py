#!/usr/bin/env python

import os
from pydub import AudioSegment


def FormatName(name):
    lower_case = "abcdefghijklmnopqrstuvwxyz"
    upper_case = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

    # Change .mp3 / mp4 to .wav
    name = name[:-4] + ".wav"

    # change to lower_case and remove empty space
    new_name = ""
    for letter in name:
        # Replace uppercase with lowercase
        if letter in upper_case:
            ind = upper_case.index(letter)
            new_name += lower_case[ind]
        elif letter == " ":
            new_name+= "_"
        else:
            new_name += letter

    return new_name

def LoadAudio(filePath):
    if filePath[-3:] == "mp3":
        return AudioSegment.from_mp3(filePath)
    elif filePath[-3:] == "mp4":
        return AudioSegment.from_file(filePath, format="mp4")

input_path = r"./input/"
output_path = r"./output/"

input_contents = os.listdir(input_path)

FailedFiles = []

for file_name in input_contents:
    try:
        file_path= os.path.join(input_path, file_name)
        save_name = FormatName(file_name)
        save_path = os.path.join(output_path, save_name)
        print(file_path, save_path)

        sound = LoadAudio(file_path)
        sound.export(save_path, format="wav")
    except:
        print("THIS FILE DIDNT CONVERT: {}".format(file_name))
        FailedFiles.append(file_name)

print("The following files failed to be converted:")
for file in FailedFiles:
    print("   -" + file)
    
    

