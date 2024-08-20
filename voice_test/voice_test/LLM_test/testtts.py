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
    

if __name__ == '__main__':
    #text = "图中有一个用黑色包装纸包着的长方体礼物盒，上面系着一个红色的蝴蝶结，放在一堆纸箱上，纸箱上有一些黑色的字迹"
    text = "你好"
    token = get_token_from_key(ak, sk)
    wav_path = "./temp/huaweitts.wav"
    print("token:", token[:8])
    b64 = stts_demo(text, token)
    base64_wav(b64, wav_path)
    speak_wav(wav_path)