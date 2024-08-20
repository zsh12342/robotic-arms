import os

# 假设您的音频文件名为 'audio.wav'，并且位于当前目录
audio_file = '/home/kuavo/lyq_dev/voice_test/sound/speak.wav'

# 使用 aplay 命令播放音频文件
# 注意：这种方式并不适用于Windows系统
os.system(f'aplay {audio_file}')