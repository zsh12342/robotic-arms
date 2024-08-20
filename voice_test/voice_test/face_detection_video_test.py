# import cv2 as cv
# import numpy as np
# def face_detect_demo(image):
#     gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
#     #face_detector = cv.CascadeClassifier("haarcascade_frontalface_default.xml")
#     face_detector = cv.CascadeClassifier("haarcascade_frontalface_alt.xml")
#     faces = face_detector.detectMultiScale(gray, 1.2, 6)
#     dynamic_x = 0
#     dynamic_y = 0
#     for x, y, w, h in faces:
#         cv.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
#         dynamic_x = x + w/2
#         dynamic_y = y + h/2
#     cv.imshow("result", image)
#     return dynamic_x,dynamic_y

# print("--------- Python OpenCV Tutorial ---------")

# capture = cv.VideoCapture("/dev/video10")
# cv.namedWindow("result", cv.WINDOW_AUTOSIZE)
# while(True):
#     ret, frame = capture.read()
#     frame = cv.flip(frame, 1)#左右翻转
#     gaze_x,gaze_y = face_detect_demo(frame)
#     print("gaze_x", gaze_x)
#     print("gaze_y", gaze_y)
#     #ask_servo to move
    
#     c = cv.waitKey(10)
#     if c == 27: # ESC
#         break
# cv.waitKey(0)
# cv.destroyAllWindows()




import cv2 as cv
import numpy as np
import configparser
from voice_synthesis import voiceSynthesis
from audio_player import AudioPlayer
import pyaudio



from zhipuai import ZhipuAI
import base64
from voice_synthesis import voiceSynthesis
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
    print("gen_glm",prompt)    
 
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
    print("remove chinese sym",messages)
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
        print("message=",question)
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

print("--------- Python OpenCV Tutorial ---------")

capture = cv.VideoCapture("/dev/video4")
cv.namedWindow("result", cv.WINDOW_AUTOSIZE)

image_path = '/home/kuavo/lyq_dev/voice_test/image/test.jpg'

def speak():
    image_b64 = image_to_base64(image_path)
    print(len(image_b64))
    question = 'answer with chinese'
    #image_b64 = 'https://img1.baidu.com/it/u=1369931113,3388870256&fm=253&app=138&size=w931&n=0&f=JPEG&fmt=auto?sec=1703696400&t=f3028c7a1dca43a080aeb8239f09cc2f'
    answer = askChatGPT(question, image_b64)
    print(answer)
    return answer



configFile = configparser.ConfigParser()
configFile.read("config.txt",encoding='utf-8')
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

AUDIO_INPUT_DEVICE_INDEX = eval(config["audio_input_device_index"])
AUDIO_INPUT_DEVICE_CHANNEL = 1
AUDIO_OUTPUT_DEVICE_INDEX = eval(config["audio_output_device_index"])
ENERGY_THREASHOLD = int(config["energy_threashold"])

RECORD_TIME = 7
RECORD_WAV_PATH = "./temp/record.wav"





while(True):
    
    ret, frame = capture.read()
    frame = cv.flip(frame, 1)  # 左右翻转
    gaze_x, gaze_y = face_detect_demo(frame)
    print("gaze_x", gaze_x)
    print("gaze_y", gaze_y)
    # ask_servo to move

    c = cv.waitKey(10)
    if c == 27:  # ESC
        break
    elif c == ord('s'):  # 按下's'键
        cv.imwrite(image_path, frame)  # 保存当前帧到指定路径
        print("Image saved to:", image_path)
        
    elif c == ord('i'):  # 按下'i'键
        audio = pyaudio.PyAudio()
        player = AudioPlayer()
        answer = speak()
        print("Image speak")
        audioFile = voiceSynthesis(
                answer, SYNTHESIS_APPID, SYNTHESIS_API_KEY, SYNTHESIS_API_SECRET
            )
        player.playPcmFile(audioFile)

cv.waitKey(0)
cv.destroyAllWindows()
