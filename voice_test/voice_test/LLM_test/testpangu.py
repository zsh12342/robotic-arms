import http.client
import ssl
import json
import requests 
import websocket
 
ak = '061NCD10JGFCV2S7KKLW'
sk = '9gNfFF7EwCAeW38vjsOLPWVmsPzU2kpqLzKiiGgD'

def get_token_from_key(access_key, secret_key):
    url = "https://iam.cn-southwest-2.myhuaweicloud.com/v3/auth/tokens" 
     
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
            "name": "cn-southwest-2"
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

messages = []
def get_messages(question):
  global messages
  messages.append({"content": question})
  return messages


def ask_pangu(question: str):
    token = get_token_from_key(ak, sk)
    global messages
    endpoint = 'pangu.cn-southwest-2.myhuaweicloud.com'
    apiservice_id = '7c996ea5-a08d-4480-9b47-29b71df679ce'
    project_id = '9af21aa345714fc3aa11fd460827e5b9'
    deployment_id = '870e96e8-08c8-4063-b0c5-b167e3942214'
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    conn = http.client.HTTPSConnection(
          endpoint, context=context)
    payload = {
          "messages": get_messages(question)
    }

    payload_json = json.dumps(payload)
    headers = {
        'X-Auth-Token': token,
        'Content-Type': 'application/json'
      }
    conn.request("POST",
                 "/v1/infers/"+ apiservice_id +"/v1/"+ project_id +"/deployments/"+ deployment_id +"/chat/completions",
                 payload_json, headers)
    print(conn)
    res = conn.getresponse()
    data = json.loads(res.read())
    print(type(data['choices']),data['choices'])
    return data['choices'][0]['message']['content']


text =[]

# length = 0

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
    


if __name__ == '__main__':
    #token = get_token_from_idpass(id, password)
    token = get_token_from_key(ak, sk)
    #token = 'iaHdfYWtfc2siXSwiY2F0YWxvZyI6W10sInJvbGVzIjpbeyJuYW1lIjoidGVfYWRtaW4iLCJpZCI6IjAifSx7Im5hbWUiOiJ0ZV9hZ2VuY3kiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9jc2JzX3JlcF9hY2NlbGVyYXRpb24iLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9lY3NfZGlza0FjYyIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Rzc19tb250aCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX29ic19kZWVwX2FyY2hpdmUiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9hX2NuLXNvdXRoLTRjIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfZGVjX21vbnRoX3VzZXIiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9pbnRsX29hIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfY2JyX3NlbGxvdXQiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9mbG93X2NhIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfZWNzX29sZF9yZW91cmNlIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfcGFuZ3UiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF93ZWxpbmticmlkZ2VfZW5kcG9pbnRfYnV5IiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfY2JyX2ZpbGUiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9sYXJnZV9tb2RlbHMiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9kbXMtcm9ja2V0bXE1LWJhc2ljIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfZG1zLWthZmthMyIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2NvY19jYSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2VkZ2VzZWNfb2J0IiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfa29vZHJpdmUiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9vYnNfZGVjX21vbnRoIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfY3Nic19yZXN0b3JlIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfaWRtZV9tYm1fZm91bmRhdGlvbiIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Vjc19jNmEiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9FQ19PQlQiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9rb29waG9uZSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX211bHRpX2JpbmQiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9zbW5fY2FsbG5vdGlmeSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2FfYXAtc291dGhlYXN0LTNkIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfcmdjIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfY3Nic19wcm9ncmVzc2JhciIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Nlc19yZXNvdXJjZWdyb3VwX3RhZyIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Vjc19vZmZsaW5lX2FjNyIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2V2c19yZXR5cGUiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9rb29tYXAiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9ldnNfZXNzZDIiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9kbXMtYW1xcC1iYXNpYyIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2V2c19wb29sX2NhIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfYV9jbi1zb3V0aHdlc3QtMmIiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9od2NwaCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Vjc19vZmZsaW5lX2Rpc2tfNCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2h3ZGV2IiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfb3BfZ2F0ZWRfY2JoX3ZvbHVtZSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX3Ntbl93ZWxpbmtyZWQiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9jY2VfYXV0b3BpbG90IiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfaHZfdmVuZG9yIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfcHJvX2NhIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfYV9jbi1ub3J0aC00ZSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX3dhZl9jbWMiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9hX2NuLW5vcnRoLTRkIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfZWNzX2hlY3NfeCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Vjc19hYzciLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9lcHMiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9jc2JzX3Jlc3RvcmVfYWxsIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfYV9jbi1ub3J0aC00ZiIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX29jdG9wdXMiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9vcF9nYXRlZF9yb3VuZHRhYmxlIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfZXZzX2V4dCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2FfYXAtc291dGhlYXN0LTFlIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfYV9ydS1tb3Njb3ctMWIiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9hX2FwLXNvdXRoZWFzdC0xZCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2FwcHN0YWdlIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfYV9hcC1zb3V0aGVhc3QtMWYiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9zbW5fYXBwbGljYXRpb24iLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9haWVkZ2VfYWxnb3JpdGhtX3BhY2thZ2UiLCJpZCI6IjAifSx7Im5hbWUiOiJvcF9nYXRlZF9jc2VfZ2F0ZXdheSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX0VDLU9CVCIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX3Jkc19jYSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Vjc19ncHVfZzVyIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfb3BfZ2F0ZWRfbWVzc2FnZW92ZXI1ZyIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2Vjc19yaSIsImlkIjoiMCJ9LHsibmFtZSI6Im9wX2dhdGVkX2FfcnUtbm9ydGh3ZXN0LTJjIiwiaWQiOiIwIn0seyJuYW1lIjoib3BfZ2F0ZWRfaWVmX3BsYXRpbnVtIiwiaWQiOiIwIn1dLCJwcm9qZWN0Ijp7ImRvbWFpbiI6eyJuYW1lIjoiaGlkX2RfNGViaWhidGQ4bTkwcyIsImlkIjoiZTVjODhiYzI5N2QyNGVmNGI2OTBmYTk2MjViNGJiYjQifSwibmFtZSI6ImNuLXNvdXRod2VzdC0yIiwiaWQiOiI5YWYyMWFhMzQ1NzE0ZmMzYWExMWZkNDYwODI3ZTViOSJ9LCJpc3N1ZWRfYXQiOiIyMDI0LTA0LTIyVDA3OjUxOjI4LjY3ODAwMFoiLCJ1c2VyIjp7ImRvbWFpbiI6eyJuYW1lIjoiaGlkX2RfNGViaWhidGQ4bTkwcyIsImlkIjoiZTVjODhiYzI5N2QyNGVmNGI2OTBmYTk2MjViNGJiYjQifSwibmFtZSI6ImhpZF9kXzRlYmloYnRkOG05MHMiLCJwYXNzd29yZF9leHBpcmVzX2F0IjoiIiwiaWQiOiIzMTcyNGJjNWVlZDY0ZDc5OTVjNDg5YjlkZDNjZTk3NyJ9fX0xggHBMIIBvQIBATCBlzCBiTELMAkGA1UEBhMCQ04xEjAQBgNVBAgMCUd1YW5nRG9uZzERMA8GA1UEBwwIU2hlblpoZW4xLjAsBgNVBAoMJUh1YXdlaSBTb2Z0d2FyZSBUZWNobm9sb2dpZXMgQ28uLCBMdGQxDjAMBgNVBAsMBUNsb3VkMRMwEQYDVQQDDApjYS5pYW0ucGtpAgkA3LMrXRBhahAwCwYJYIZIAWUDBAIBMA0GCSqGSIb3DQEBAQUABIIBABcO1OC6MpUdqah6ScDhRB9nXHJgZp0Ew4DW3ckPDjgmoSHmTZ2YNg+HtxkBRhACQJtHgB0q6mxv12pWn-9vDkpSEEVOZQIsVSyW+ZMcc2pOgoouRu2dMOScuux5r4xyWP5M5qZWxlrDqPYZvRaChowLCyPU++3yX1JE+2MkZWaXfyQ1gPZs17QeQwy8PefPA3zsil93U43phuTD-AH32-mplS5m5VVpLu6kMzT01X23iqV3LlA06yMu8gsEaXdv2WnI7B2eqj+p4xH-2DuokhvrziKaktX48ansKDaeDwg-dPWo+V7hHhkFR5SgWnMYmB1Heq758uiX7lpZAf2bdRg='
    print(token[:8])
    text.clear
    question = """
        你是具身智能大脑，现在你需要感知图片内容并理解我的问题，输出答案来指挥语音模块和机器人，输出格式为json格式，{"vocie": xxxx, "robot": 步骤1：xx， 步骤2：xx}。其中"voice"字段中输出的内容是由语音模块输出给提出问题的人的答案，"robot"字段中输出的内容是给机器人的执行步骤规划。 下面是几个示例 
      
        示例一： 问题：请问图片中有什么? 回答：{"voice": 图片中有一个冰箱，一个橱柜，冰箱中有七喜饮料，可能是在厨房中, "robot": 无需做出规划动作}\n
        示例二： 问题：我渴了，能帮帮我吗？ 回答：{"voice": 当然可以，因为冰箱中有七喜饮料，所以我可以给你拿一瓶饮料, "robot": 步骤1：移动到冰箱旁边；步骤2：打开冰箱；步骤3：拿起饮料；步骤4：把饮料放到主人手上；步骤5：关闭冰箱；步骤6：完成}\n
        
        图片中有三个橘子放在碗中，碗在一张桌子上，旁边有个椅子，还有榨汁机，这是一个厨房场景.现在请开始理解我的问题并做出回答：图片中有什么？
    """
    print('question:',question)
    while(1):
        #print("pangu:",end = "")
        answer = ask_pangu(question, token)
        answer_content = answer["choices"][0]["message"]["content"]
        get_messages(answer_content)
        print(answer_content)
        print("===================>")
        # question = checklen(Input)

        question = input("输入的问题：")       
        # print(str(text))
