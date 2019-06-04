import wave
import sys
import os

def parse_wave(file_path):
    Wave_read = wave.open(file_path, mode = 'rb')

    # Display file info
    print('Number of channels:', Wave_read.getnchannels())
    print('Sample width:', Wave_read.getsampwidth())
    print('Sampling frequency:', Wave_read.getframerate())
    n_frames = Wave_read.getnframes()
    print('Number of audio frames:', n_frames)

    audio_data = Wave_read.readframes(n_frames)
    audio_data_len = len(audio_data)
    print('Number of audio bytes:', audio_data_len)
    
    output_file = open(file_path + '_output', 'w') 

    for i in range(audio_data_len):
        output_file.write(str(audio_data[i]) + ',\n') 
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: <file_path> where <file_path> is the path to a .wav file')
        sys.exit(1)
    
    parse_wave(sys.argv[1])