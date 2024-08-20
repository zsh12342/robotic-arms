# -*- coding: utf-8 -*-
# 此demo仅供测试使用，强烈建议使用sdk

import json
import requests
import base64
ak = '061NCD10JGFCV2S7KKLW'
sk = '9gNfFF7EwCAeW38vjsOLPWVmsPzU2kpqLzKiiGgD'
token_url_dict = {
    "pangu": "https://iam.cn-southwest-2.myhuaweicloud.com/v3/auth/tokens",
    "voice": "https://iam.cn-north-4.myhuaweicloud.com/v3/auth/tokens"
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

def sasr_demo():
    url = 'https://{{endpoint}}/v1/{{project_id}}/asr/short-audio'  # endpoint和project_id需替换
    token = '用户对应region的token'
    file_path = '将要识别音频的路径'
    with open(file_path, 'rb') as f:
        data = f.read()
        base64_data = str(base64.b64encode(data), 'utf-8')
    header = {
        'Content-Type': 'application/json',
        'X-Auth-Token': token
    }
    body = {
        'data': base64_data,
        'config': {
            'property': 'chinese_8k_common',
            'audio_format': 'pcm8k16bit'
        }
    }
    resp = requests.post(url, data=json.dumps(body), headers=header)
    print(resp.text)

if __name__ == '__main__':
    token = get_token_from_key(ak, sk)
    print(token)
    # rasr_demo()

