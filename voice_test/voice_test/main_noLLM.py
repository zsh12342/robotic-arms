# config microphone: Bothlent UAC Dongle: USB Audio (hw:1,0) inputchannel: 8
# config speaker: 输入设备 ID  0  -  USB Audio Device: - (hw:0,0) inputchannel: 1
import json
import pyaudio
import speech_recognition as sr
import os
import configparser
import pygame
import time
import yaml
import rospy

from voice_record import recordVoice
from voice_recognition import voiceRecognition
from voice_synthesis import voiceSynthesis
from audio_tool import wav2pcm
from audio_device import listAudioDevice
from audio_player import AudioPlayer
from zhipuai import ZhipuAI
import re

from abilities.action_control import action_control
import asyncio

configFile = configparser.ConfigParser()
configFile.read("./config/config.txt",encoding='utf-8')
config = dict(configFile.items("config"))

#OPENAI_API_KEY = config["openai_api_key"]
NETWORK_PROXY = config["network_proxy"]

RECOGNITION_APPID = config["recognition_app_id"]
RECOGNITION_API_KEY = config["recognition_api_key"]
RECOGNITION_API_SECRET = config["recognition_api_secret"]
RECOGNITION_FILE_PATH = "./temp/record.pcm"

SYNTHESIS_APPID = config["synthesis_app_id"]
SYNTHESIS_API_KEY = config["synthesis_api_key"]
SYNTHESIS_API_SECRET = config["synthesis_api_secret"]
SYNTHESIS_FILE_PATH = "./temp/synthesis.pcm"

ZHIPUAI_API_KEY = config["zhipuai_api_key"]
client = ZhipuAI(
    api_key = f"{ZHIPUAI_API_KEY}"
)

AUDIO_INPUT_DEVICE_INDEX = eval(config["audio_input_device_index"])
AUDIO_INPUT_DEVICE_CHANNEL = 1
AUDIO_OUTPUT_DEVICE_INDEX = eval(config["audio_output_device_index"])
ENERGY_THREASHOLD = int(config["energy_threashold"])

RECORD_TIME = 7
RECORD_WAV_PATH = "./temp/record.wav"

text =[]
count = 0
question = " "
answer = " "
audio = pyaudio.PyAudio()
player = AudioPlayer()

# robot action state
RECORD_STATE = 0  # 录音状态
VOICE_NULL = 1    # 无语音之后的状态
VOICE_FULL = 2    # 有语音之后的状态（检索）
ACTION_NO = 3    # 有语音之后的状态，检索没有技能集需要实现
ACTION_IS = 4    # 有语音之后的状态，检索有技能集需要实现
PLAY_ANSWER = 5  # 播放语音
robotState = RECORD_STATE
break_Sign = 0

# 是否开启 技能集 配置
abilities_flag = True

# 读取 ability.yaml 文件
def load_abilities(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        abilities = yaml.safe_load(file)
    return abilities['abilities']

# 检查 question 是否包含技能集的关键字
def check_abilities(question, abilities):
    for ability, keywords in abilities.items():
        for keyword in keywords:
            if keyword in question:
                return ability, keyword
    return None, None

def recordVoiceSmart(mic: sr.Microphone, save_file="record.pcm", timeout=10):
    r = sr.Recognizer()
    r.energy_threshold = ENERGY_THREASHOLD

    print("Listening....")

    try:
        audio = r.listen(mic, timeout=timeout)
        with open(save_file, "wb") as f:
            f.write(audio.get_wav_data())
        print("Record complete.")
        return True  # 表示成功录音
    except sr.WaitTimeoutError:
        print("Listening timed out, no voice detected.")
        return False  # 表示没有检测到语音


def voiceToText(audio: pyaudio.PyAudio=None, mic: sr.Microphone=None, smart=True):
    if smart:
        result = recordVoiceSmart(mic, RECOGNITION_FILE_PATH)
        # 异常检测
        if not result:
             return " "
        print("smart")
    else:
        recordVoice(audio, RECORD_TIME, RECORD_WAV_PATH, AUDIO_INPUT_DEVICE_INDEX)
        wav2pcm(RECORD_WAV_PATH, RECOGNITION_FILE_PATH)
        print("not smart")

    recognition = voiceRecognition(
        RECOGNITION_FILE_PATH,
        RECOGNITION_APPID,
        RECOGNITION_API_KEY,
        RECOGNITION_API_SECRET,
    )

    return recognition

def getText(role,content):
    jsoncon = {}
    jsoncon["role"] = role
    jsoncon["content"] = content
    text.append(jsoncon)
    return text

def getlength(text):
    length = 0
    for content in text:
        temp = content["content"]
        leng = len(temp)
        length += leng
    return length

def checklen(text):
    while (getlength(text) > 8000):
        del text[0]
    return text
    

# 读取 prompt 文件
def read_prompt_from_file(prompt_file):
    with open(prompt_file, 'r', encoding='utf-8') as f:
        prompt_text = f.read().strip()
    return prompt_text


def remove_periods(s: str):
    # 删除字符串中的所有句号
    s.replace('！', '')
    s.replace('？', '')
    s.replace('。', '')
    s.replace('，', '')
    return s

def calibrateEnergyThreashold(mic: sr.Microphone):
    rec = sr.Recognizer()
    rec.adjust_for_ambient_noise(mic)
    print("energy_threashold: ", rec.energy_threshold)

def remove_some_rules(word: str):
    """
    返回的 answer 有两种形式，需要进行字符处理
    [1] {role=assistant, content=珠穆朗玛峰，也被称为珠峰，是地球上最高的山峰，海拔高度为8,848.86米（根据2020年的测量数据）。它位于中国与尼泊尔边界的喜马拉雅山脉中。}
    [2] 珠穆朗玛峰，又称珠穆朗玛峰，是世界上最高的山峰，位于中国与尼泊尔的边界上。它的海拔高度为8,848米。这座峰以其壮丽的自然风光和攀登挑战而闻名于世。
    """
    # 定义匹配 {role=assistant, content=...} 的正则表达式
    pattern = r'\{role=[^,]+, content=([^}]+)\}'
    
    # 使用正则表达式查找匹配的内容
    match = re.search(pattern, word)
    
    if match:
        # 如果匹配到，则返回 content 中的内容
        cleaned_word = match.group(1).strip()
    else:
        # 如果没有匹配到，则返回原始字符串
        cleaned_word = word.strip()
    
    # 移除一些特殊符号，如*号
    cleaned_word = cleaned_word.replace('*', '')
    cleaned_word = cleaned_word.replace('**', '')  # 如果有连续的*号，也一并移除
    
    return cleaned_word
    
# ---------------------------------- 状态层 ------------------------------------ # 
def recordVoice_wav():
    """
        录制音频状态
    """
    global question
    global count
    global robotState

    player.playSoundList("speak")
    player.waitForPlayer()

    with sr.Microphone(AUDIO_INPUT_DEVICE_INDEX, 16000) as mic:
        if count == 0:
            print("[Log] Audio initialization compelete.")

            print("[Audio Device List]")
            listAudioDevice(audio)
            print()

            print("[Measure Ambient Noise]")
            calibrateEnergyThreashold(mic)
            print()

            print("[Start ChatGPT]")         

        # 先清空，然后转语音
        question = " "
        question = voiceToText(audio=audio, mic=mic)
        a_result = type(question)
        print(a_result)
        print("> ", question)
        
        # 切换状态
        if not question.strip():
            print(" -- 没收到任何语音 -- ")
            robotState = VOICE_NULL
        else:
            print(" --  接收到语音  -- ")
            robotState = VOICE_FULL

    # 打印设备列表计数器
    count += 1

def noVoice():
    """
        无语音之后的状态
    """
    global robotState
    print(" ---- robotState: 无语音之后的状态 ----------")
    time.sleep(15)
    robotState = RECORD_STATE

def isVoice():
    """
        有语音的状态
    """
    global question
    global abilities_flag
    global robotState
    print(" ---- robotState: 有语音的状态 (检索是否包含技能集) ----------")
    
    if abilities_flag:
        print("开启人机交互 | 技能集配置 | 语音交互")
        robotState = ACTION_IS
    else:
        print("开启人机交互 | 语音交互")
        robotState = ACTION_NO

def voice_no_action():
    """
        有语音之后的状态，检索没有技能集需要实现
    """
    global question
    global answer
    global robotState

    answer = remove_some_rules(answer)
    print(type(answer))
    print(answer)
    print()

    # 文本转语音合成
    robotState = PLAY_ANSWER

async def run_action_control(ability_name: str):
    await action_control(ability_name)

def voice_is_action():
    """
        有语音之后的状态，检索有技能集需要实现
    """
    global question
    global answer
    global robotState

    # 检索question里面是否包含了技能集的话语
    abilities = load_abilities("./config/ability.yaml")
    matched_ability, matched_keyword = check_abilities(question, abilities)

    if matched_ability:
        # 匹配到技能集，执行技能集里面的固定动作/播放固定的语音
        print(f"匹配到技能集: {matched_ability}，关键字: {matched_keyword}")
        asyncio.create_task(run_action_control(matched_ability))
        answer = str(f"好啊，我给大家{matched_keyword}吧")
        print(" 匹配到技能集，执行技能集 answer type : " ,type(answer))
    else:
        # 没匹配到技能集，直接回复没有匹配到机器人的技能
        answer = str(f"没有匹配到任何机器人技能")

    # 文本转语音合成
    robotState = PLAY_ANSWER

def voice_play_answer():
    global robotState
    global answer
    global player

    audioFile = voiceSynthesis(
        answer, SYNTHESIS_APPID, SYNTHESIS_API_KEY, SYNTHESIS_API_SECRET
    )
    player.playPcmFile(audioFile)

    print("[Log] Dialog complete.")
    input("Press Enter to continue...")

    robotState = RECORD_STATE

stateActionMap = {
    RECORD_STATE:recordVoice_wav,    # 录制语音(检索)
    VOICE_NULL:noVoice,          # 无语音之后的状态
    VOICE_FULL:isVoice,          # 有语音之后的状态（检索）
    ACTION_NO:voice_no_action,   # 有语音之后的状态，检索没有技能集需要实现
    ACTION_IS:voice_is_action,   # 有语音之后的状态，检索有技能集需要实现
    PLAY_ANSWER:voice_play_answer  # 播放语音
}

async def doing_voice_job():
    global robotState
    global break_Sign
    while not rospy.is_shutdown():
        # 动作空间执行
        stateActionMap.get(robotState)()
        # 全局退出
        if break_Sign == 1:
            break
        await asyncio.sleep(0.1)  # 防止阻塞事件循环

async def main():
    await doing_voice_job()
    pygame.quit()


if __name__ == "__main__":
    rospy.init_node('kuavo_robot_Embodied_AInode')
    asyncio.run(main())

