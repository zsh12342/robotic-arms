import cv2 as cv
import speech_recognition as sr
import configparser
import requests
import json
import base64
import os
import time
from zhipuai import ZhipuAI
import base64
from voice_record import recordVoice
from audio_tool import wav2pcm
from voice_recognition import voiceRecognition
from testpangu import ask_pangu
import pyaudio
from audio_player import AudioPlayer



configFile = configparser.ConfigParser()
configFile.read("config.txt",encoding='utf-8')
config = dict(configFile.items("config"))

RECOGNITION_APPID = config["recognition_app_id"]
RECOGNITION_API_KEY = config["recognition_api_key"]
RECOGNITION_API_SECRET = config["recognition_api_secret"]
RECOGNITION_FILE_PATH = "./temp/record.pcm"
AUDIO_INPUT_DEVICE_INDEX = eval(config["audio_input_device_index"])
AUDIO_INPUT_DEVICE_CHANNEL = 1
AUDIO_OUTPUT_DEVICE_INDEX = eval(config["audio_output_device_index"])
ENERGY_THREASHOLD = int(config["energy_threashold"])

RECORD_TIME = 7
RECORD_WAV_PATH = "./temp/record.wav"


def recordVoiceSmart(mic: sr.Microphone, save_file="record.pcm", timeout=10):
    r = sr.Recognizer()
    r.energy_threshold = ENERGY_THREASHOLD

    print("Listening....")

    audio = r.listen(mic, timeout)
    with open(save_file, "wb") as f:
        f.write(audio.get_wav_data())
        #print(audio.get_wav_data())
        #f.close()
        

    print("Record complete.")


def voiceToText(audio: pyaudio.PyAudio=None, mic: sr.Microphone=None, smart=True):
    if smart:
        recordVoiceSmart(mic, RECOGNITION_FILE_PATH)
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
client = ZhipuAI(
    #api_key = "2ed622a64cd59489557bf88ab570bc1e.hdWhRouuja0IFy1S"
    #api_key = "96a303b8a3befeb9149eb3ac94cde00b.oRbTSvV8oY2nrHRc"
    api_key = "073d8e599b11d4ccdaec7ede6c41b271.FrEKFwruHCrJkF1n"
    #api_key=os.environ["ZHIPUAI_API_KEY"]
)

import re

# def remove_chinese_symbols(s):
#     # 定义中文符号的正则表达式
#     chinese_symbols = r'[，。！？；：“”（）【】《》、]'
#     # 使用正则表达式删除中文符号
#     result = re.sub(chinese_symbols, '', s)
#     return result

image_path = '/home/kuavo/lyq_dev/voice_test/image/test.jpg'

def face_detect_demo(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    face_detector = cv.CascadeClassifier("haarcascade_frontalface_alt.xml")
    faces = face_detector.detectMultiScale(gray, 1.2, 6)
    dynamic_x = 0
    dynamic_y = 0
    for x, y, w, h in faces:
        cv.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
        dynamic_x = x + w/2
        dynamic_y = y + h/2
    cv.imshow("result", image)
    return dynamic_x, dynamic_y

def remove_periods(s: str):
    # 删除字符串中的所有句号
    s.replace('！', '')
    s.replace('？', '')
    s.replace('。', '')
    s.replace('，', '')
    return s



def gen_glm_params(prompt: str):
    '''
    构造 GLM 模型请求参数 messages

    请求参数：
        prompt: 对应的用户提示词
    '''
    #cleaned_prompt = remove_chinese_symbols(prompt)
    #cleaned_prompt = prompt[:-1]
    print("prompt type : ", type(prompt))
    
    ## process
    if str(type(prompt)) == "<class 'list'>":
        prompt = prompt.pop()
    else:
        prompt = remove_periods(prompt)
        #prompt = prompt[:-1]
        
        
    ## 
    #print("gen_glm",prompt)    
 
    return [prompt]


def get_completion(prompt: str, model="glm-4v", temperature=0.95):
    '''
    获取 GLM 模型调用结果

    请求参数：
        prompt: 对应的提示词
        model: 调用的模型，默认为 glm-4，也可以按需选择 glm-3-turbo 等其他模型
        temperature: 模型输出的温度系数，控制输出的随机程度，取值范围是 0~1.0，且不能设置为 0。温度系数越低，输出内容越一致。
    '''
    
    messages = gen_glm_params(prompt)
    #print("remove chinese sym",messages)
    response = client.chat.completions.create(
        model=model,
        messages=messages,
        temperature=temperature
    )
    if len(response.choices) > 0:
        return response.choices[0].message.content
    return "generate answer error"

def askChatGPT(question: str, image=None):
    #Input = input("\n" +"我:")
    if image:
        question = getTextAndImage("user",question, image)
        #print("message=",question)
    else:
        question = checklen(getText("user",question))
    answer_zhipu = get_completion(question)

    return answer_zhipu
text = []
def getText(role,content):
    jsoncon = {}
    jsoncon["role"] = role
    jsoncon["content"] = content
    text.append(jsoncon)
    return text

def getTextAndImage(role, text, image):
    messages = []
    jsoncon = {}
    content = []
    text_dict = {}
    text_dict["type"] = "text"
    text_dict["text"] = text
    content.append(text_dict)
    image_dict = {}
    image_dict["type"] = "image_url"
    image_dict["image_url"] = {"url":image}
    content.append(image_dict)
    
    jsoncon["role"] = role
    jsoncon["content"]= content
    messages.append(jsoncon)
    return messages

def getlength(text):
    length = 0
    for content in text:
        temp = content["content"]
        leng = len(temp)
        length += leng
    return length

def checklen(text):
    while (getlength(text) > 800000):
        del text[0]
    return text
    
def image_to_base64(image_path):
    with open(image_path, "rb") as image:
        encoded_str = base64.b64encode(image.read())
    return encoded_str.decode('utf-8')
def ask_image(image_path,question):
    image_b64 = image_to_base64(image_path)
    print(len(image_b64))
    #image_b64 = 'https://img1.baidu.com/it/u=1369931113,3388870256&fm=253&app=138&size=w931&n=0&f=JPEG&fmt=auto?sec=1703696400&t=f3028c7a1dca43a080aeb8239f09cc2f'
    #image_b64 = ""
    answer = askChatGPT(question, image_b64)
    print(answer)
    return answer

end_point = "sis-ext.cn-east-3.myhuaweicloud.com"

ak = '061NCD10JGFCV2S7KKLW'
sk = '9gNfFF7EwCAeW38vjsOLPWVmsPzU2kpqLzKiiGgD'
token_url_dict = {
    "pangu": "https://iam.cn-southwest-2.myhuaweicloud.com/v3/auth/tokens",
    "voice": "https://iam.cn-north-4.myhuaweicloud.com/v3/auth/tokens",
    "tts": "https://iam.cn-east-3.myhuaweicloud.com/v3/auth/tokens"
}
def get_token_from_key(access_key, secret_key):
    url = "https://iam.cn-east-3.myhuaweicloud.com/v3/auth/tokens"
     
    payload = json.dumps({ 
      "auth": { 
        "identity": { 
          "methods": [ 
            "hw_ak_sk" 
          ], 
          "hw_ak_sk": { 
            "access": { 
              "key": access_key
            }, 
            "secret": { 
              "key": secret_key
            } 
          } 
        }, 
        "scope": { 
          "project": { 
            "name": "cn-east-3"
          } 
        } 
      } 
    }) 
    headers = { 
      'Content-Type': 'application/json' 
    } 
     
    response = requests.request("POST", url, headers=headers, data=payload) 
     
    token = response.headers["X-Subject-Token"]
    return token

def stts_demo(text, token):
    #url = 'https://'+ end_point +'/v1/{{project_id}}/tts'  # endpoint和project_id需替换
    url = 'https://sis-ext.cn-east-3.myhuaweicloud.com/v1/0d7978fb689c4a7e96a4d8e145c13bdc/tts'
    #token = '用户对应region的token'
    #text = '待识别的文本'
    header = {
        'Content-Type': 'application/json',
        'X-Auth-Token': token
    }
    body = {'text': text}
    resp = requests.post(url, data=json.dumps(body), headers=header)
    res_dict = json.loads(resp.text)
    b64_str = res_dict["result"]["data"]
    print(b64_str[:8])
    return b64_str

def base64_wav(b64str, wav_path):
    # 将 base64 字符串解码为二进制数据
    audio_data = base64.b64decode(b64str)
    
    # 指定输出文件名
    output_filename = wav_path
    
    # 将二进制数据写入到 .wav 文件
    with open(output_filename, 'wb') as audio_file:
        audio_file.write(audio_data)
    
    print(f'文件已保存为: {output_filename}')

def speak_wav(wav_path):
    os.system(f'aplay {wav_path}')
def say_text(text):
    token = get_token_from_key(ak, sk)
    wav_path = "./temp/huaweitts.wav"
    print("token:", token[:8])
    b64 = stts_demo(text, token)
    base64_wav(b64, wav_path)
    speak_wav(wav_path)

def say_image(question,frame):
    image_path = '/home/kuavo/lyq_dev/voice_test/image/test.jpg'
    
        #frame = cv.flip(frame, 1)  # 左右翻转
        #gaze_x, gaze_y = face_detect_demo(frame)
        # time.sleep(1)
        # c = cv.waitKey(10)
        # if c == ord('s'):  # 按下's'键
        #     break
    cv.imwrite(image_path, frame)  # 保存当前帧到指定路径
    print("Image saved to:", image_path)

    #text = "图中有一个用黑色包装纸包着的长方体礼物盒，上面系着一个红色的蝴蝶结，放在一堆纸箱上，纸箱上有一些黑色的字迹"
    text = ask_image(image_path,question)
    print(text)
    say_text(text)
    return text

def say_pangu(question):
    answer = ask_pangu(question)
    print(answer)
    say_text(answer)
    
def dialogue_history(question_answer,question):
    answer = question_answer + question
    return answer

def calibrateEnergyThreashold(mic: sr.Microphone):
    rec = sr.Recognizer()
    rec.adjust_for_ambient_noise(mic)
    print("energy_threashold: ", rec.energy_threshold)
    
def speak_wav(wav_path):
    os.system(f'aplay {wav_path}')

def remove_periods(s: str):
    # 删除字符串中的所有句号
    s.replace('！', '')
    s.replace('？', '')
    s.replace('。', '')
    s.replace('，', '')
    return s

import requests
import json
import base64
import os

end_point = "sis-ext.cn-east-3.myhuaweicloud.com"

ak = '061NCD10JGFCV2S7KKLW'
sk = '9gNfFF7EwCAeW38vjsOLPWVmsPzU2kpqLzKiiGgD'
token_url_dict = {
    "pangu": "https://iam.cn-southwest-2.myhuaweicloud.com/v3/auth/tokens",
    "voice": "https://iam.cn-north-4.myhuaweicloud.com/v3/auth/tokens",
    "tts": "https://iam.cn-east-3.myhuaweicloud.com/v3/auth/tokens"
}
def get_token_from_key(access_key, secret_key):
    url = "https://iam.cn-east-3.myhuaweicloud.com/v3/auth/tokens"
     
    payload = json.dumps({ 
      "auth": { 
        "identity": { 
          "methods": [ 
            "hw_ak_sk" 
          ], 
          "hw_ak_sk": { 
            "access": { 
              "key": access_key
            }, 
            "secret": { 
              "key": secret_key
            } 
          } 
        }, 
        "scope": { 
          "project": { 
            "name": "cn-east-3"
          } 
        } 
      } 
    }) 
    headers = { 
      'Content-Type': 'application/json' 
    } 
     
    response = requests.request("POST", url, headers=headers, data=payload) 
     
    token = response.headers["X-Subject-Token"]
    return token

def stts_demo(text, token):
    #url = 'https://'+ end_point +'/v1/{{project_id}}/tts'  # endpoint和project_id需替换
    url = 'https://sis-ext.cn-east-3.myhuaweicloud.com/v1/0d7978fb689c4a7e96a4d8e145c13bdc/tts'
    #token = '用户对应region的token'
    #text = '待识别的文本'
    header = {
        'Content-Type': 'application/json',
        'X-Auth-Token': token
    }
    body = {'text': text}
    resp = requests.post(url, data=json.dumps(body), headers=header)
    res_dict = json.loads(resp.text)
    b64_str = res_dict["result"]["data"]
    print(b64_str[:8])
    return b64_str

def base64_wav(b64str, wav_path):
    # 将 base64 字符串解码为二进制数据
    audio_data = base64.b64decode(b64str)
    
    # 指定输出文件名
    output_filename = wav_path
    
    # 将二进制数据写入到 .wav 文件
    with open(output_filename, 'wb') as audio_file:
        audio_file.write(audio_data)
    
    print(f'文件已保存为: {output_filename}')

def speak_wav(wav_path):
    os.system(f'aplay {wav_path}')
    

# if __name__ == '__main__':
#     #text = "图中有一个用黑色包装纸包着的长方体礼物盒，上面系着一个红色的蝴蝶结，放在一堆纸箱上，纸箱上有一些黑色的字迹"
#     text = "你好"
#     token = get_token_from_key(ak, sk)
#     wav_path = "./temp/huaweitts.wav"
#     print("token:", token[:8])
#     b64 = stts_demo(text, token)
#     base64_wav(b64, wav_path)
#     speak_wav(wav_path)







if __name__ == '__main__':
    
    
    player = AudioPlayer()
    count = 0
    dialogue_flag = 0
    answer_image = " "

    #capture = cv.VideoCapture("/dev/video4")
    #cv.namedWindow("result_visual", cv.WINDOW_AUTOSIZE)
    while True:
        with sr.Microphone(AUDIO_INPUT_DEVICE_INDEX, 16000) as mic:
            audio = pyaudio.PyAudio()
            wake_up_word = voiceToText(audio=audio, mic=mic)
            if "夸父" in wake_up_word:
                print("收到唤醒词")
                text = "你好,我是你的家庭服务助手夸父，请问你需要我做什么"
                token = get_token_from_key(ak, sk)
                wav_path = "./temp/huaweitts.wav"
                print("token:", token[:8])
                b64 = stts_demo(text, token)
                base64_wav(b64, wav_path)
                speak_wav(wav_path)
                break
    capture = cv.VideoCapture("/dev/video4")
    cv.namedWindow("result", cv.WINDOW_AUTOSIZE)    
    while True:
        #player.playSoundList("speak")
        ret, frame = capture.read()
        cv.imshow("result", frame)
        with sr.Microphone(AUDIO_INPUT_DEVICE_INDEX, 16000) as mic:
            audio = pyaudio.PyAudio()
            if count == 0:
                print("[Measure Ambient Noise]")
                calibrateEnergyThreashold(mic)
                ## process
            question = voiceToText(audio=audio, mic=mic)
            if str(type(question)) == "<class 'list'>":
                question = question.pop()
            else:
                question = remove_periods(question)
                #prompt = prompt[:-1]    
            #question = "what is your name "
            a_result = type(question)
            print(a_result)
            print("> ", question)
            if "看到" in question:
                question += "你看到了什么？ 请按照如下句式回答，我看到了<...>"
                answer_image = say_image(question,frame)
            # if "名字" in question:
            #     print("")
                
            else:
                question = answer_image + question + "请用简洁语言回答"
                say_pangu(question)
                
                
            # cv.destroyAllWindows()
            # cv.waitKey(0)
            # cv.destroyAllWindows()
        
        count += 1
        c = cv.waitKey(10)
        if c == 27:  # ESC
            break
    cv.waitKey(0)
    cv.destroyAllWindows()
                

