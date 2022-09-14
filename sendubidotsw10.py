import time
import requests
import os
from dotenv import load_dotenv
import random



load_dotenv()

# TOKEN = "..."  # Put your TOKEN here
TOKEN = os.getenv('TOKEN') # Put your TOKEN here
DEVICE_LABEL = os.getenv('DEVICE_LABEL')  # Put your device label here 
VARIABLE_LABEL_1 = os.getenv('VARIABLE_LABEL_1')  
VARIABLE_LABEL_2 = os.getenv('VARIABLE_LABEL_2')


VARIABLE_ID = os.getenv('VARIABLE_ID')  # Put your first variable label here


def build_payload(variable_1, variable_2,
                    value_1, value_2):

    payload = {
                variable_1: value_1, 
                variable_2: value_2
              }

    return payload

def post_request(payload):
    # Creates the headers for the HTTP requests
    try:
        url = "http://industrial.api.ubidots.com/api/v1.6/devices/"+ DEVICE_LABEL +"/" 
        headers = {"X-Auth-Token": TOKEN, "Content-Type": "application/json"}

        # Makes the HTTP requests
        req = requests.post(url=url, headers=headers, json=payload)
        status = req.status_code
        time.sleep(1)

        # Processes results
        # print(req.status_code, req.json())
        if status >= 400:
            print("[ERROR] Could not send data after 5 attempts, please check \
                your token credentials and internet connection")
            pass

        print("[INFO] request made properly, your device is updated")
        return True
    except:
        print("Cannot send to ubidots")
        pass




def sendData(value_1, value_2):
    '''
    Send data to Ubidots
    '''

    payload = build_payload(
        VARIABLE_LABEL_1, VARIABLE_LABEL_2,  value_1, value_2)

    print("[INFO] Attemping to send data")

    attempt = 0
    # print(payload)

    for attempt in range(1,6):
        print(f'[INFO] Attempt #{attempt}')
        a = post_request(payload)

        if a:

            print('[INFO] Data sent:')
            print(f'[INFO] {VARIABLE_LABEL_1}: {value_1}')
            print(f'[INFO] {VARIABLE_LABEL_2}: {value_2}')
            
            print("[INFO] finished")

            return True
        attempt += 1
    print('Failed to send data after 5 attempt!')
    return False

if __name__ == '__main__':
    while True:
        heartrate= random.randint(60,100)
        spo2= random.randint(95,100)
        sendData(heartrate,spo2)
        time.sleep(3)