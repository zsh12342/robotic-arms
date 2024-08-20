import pyaudio
import wave
import os


def recordVoice(p: pyaudio.PyAudio, time=7, save_file="record.wav", device_index=1):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = time  # 需要录制的时间
    WAVE_OUTPUT_FILENAME = save_file  # 保存的文件名

    print("Listening...")

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index=device_index,
                    frames_per_buffer=CHUNK)  # 创建录音文件
    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)  # 开始录音

    print("Record complete.")

    stream.stop_stream()
    stream.close()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')  # 保存
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()


if __name__ == '__main__':
    FFMPEG_PATH = "ffmpeg"
    WAV_NAME = "record.wav"
    PCM_NAME = "record.pcm"
    p = pyaudio.PyAudio()  # 初始化
    recordVoice(p)
    p.terminate()
    command = "{} -y -i {} -acodec pcm_s16le -f s16le -ac 1 -ar 16000 {}".format(FFMPEG_PATH, WAV_NAME, PCM_NAME)
    #print(command)
    os.system(command)
# ffmpeg -y -i test.wav -acodec pcm_s16le -f s16le -ac 1 -ar 16000 test.pcm
