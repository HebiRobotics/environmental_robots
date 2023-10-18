# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import os
import socket
import json
import threading
import time
import subprocess

import matplotlib.pyplot as plt

HOST = '127.0.0.1'  # The server's hostname or IP address
DATA_PORT = 54321        # The port used by the server
DEBUG_PORT = 65432
# Press the green button in the gutter to run the script.

class PSDSettings:
    scanTime = 0
    commonWaveNum = 0
    opticalGain = 0
    apodizationSel = 0
    zeroPadding = 0
    mode = 0

class CommunicationJson:
    scannerId = 0
    operationId = 0
    scanNum = 0
    scanSettings = {}
    gainSetting = 0
    lampsCount = 2
    lampsSelect = 0
    t1 = 14
    deltaT = 2
    t2C1 = 5
    t2C2 = 35
    t2max = 10


def convertToJson(comJson):
    return f'{json.dumps(comJson.__dict__)}\n'.encode(encoding='utf-8')


def sendRequest(requestJson, dataSocket):
    dataSocket.send(convertToJson(requestJson))
    length_bytes = dataSocket.recv(8)
    length = int(length_bytes)
    response = bytearray()
    while length > 0:
        current_packet = dataSocket.recv(4096)
        response.extend(current_packet)
        length -= len(current_packet)
    response = response.decode(encoding='utf-8')
    return response


def setSpectrumSettings(comJson1 : CommunicationJson, data_socket):
    #define source settings
    comJson1.lampsCount = 2
    comJson1.lampsSelect = 0
    comJson1.t1 = 14
    comJson1.deltaT = 2
    comJson1.t2C1 = 5
    comJson1.t2C2 = 35
    comJson1.t2max = 10

    # 1) Run set source settings
    comJson1.operationId = 22
    sendRequest(comJson1,data_socket)

    # 2) set Optical Settings
    comJson1.gainSetting = 0
    comJson1.operationId = 23
    sendRequest(comJson1,data_socket)

    #define Scan settings
    scanSettingsDict = PSDSettings()
    scanSettingsDict = PSDSettings()
    scanSettingsDict.scanTime = 10000  # millis seconds
    scanSettingsDict.commonWaveNum = 3  # supported from 0 -> 5
    scanSettingsDict.opticalGain = 0
    scanSettingsDict.apodizationSel = 0
    scanSettingsDict.zeroPadding = 1
    scanSettingsDict.mode = 0
    scanSettingsDict = scanSettingsDict.__dict__
    comJson1.scanSettings = scanSettingsDict

def runBackground(comJson1 : CommunicationJson,data_socket):
    # 3) run background
    comJson1.operationId = 11
    return sendRequest(comJson1, data_socket)

def runMaterial(comJson1 : CommunicationJson,data_socket):
    # 4) Run Spectrum
    comJson1.operationId = 12
    return sendRequest(comJson1, data_socket)

def runSpectrumFlow(comJson1 : CommunicationJson,data_socket):

    #define source settings
    comJson1.lampsCount = 2
    comJson1.lampsSelect = 0
    comJson1.t1 = 14
    comJson1.deltaT = 2
    comJson1.t2C1 = 5
    comJson1.t2C2 = 35
    comJson1.t2max = 10

    # 1) Run set source settings
    comJson1.operationId = 22
    sendRequest(comJson1,data_socket)

    # 2) set Optical Settings
    comJson1.gainSetting = 0
    comJson1.operationId = 23
    sendRequest(comJson1,data_socket)

    #define Scan settings
    scanSettingsDict = PSDSettings()
    scanSettingsDict = PSDSettings()
    scanSettingsDict.scanTime = 10000  # millis seconds
    scanSettingsDict.commonWaveNum = 3  # supported from 0 -> 5
    scanSettingsDict.opticalGain = 0
    scanSettingsDict.apodizationSel = 0
    scanSettingsDict.zeroPadding = 1
    scanSettingsDict.mode = 0
    scanSettingsDict = scanSettingsDict.__dict__
    comJson1.scanSettings = scanSettingsDict

    # 3) run background
    comJson1.operationId = 11
    sendRequest(comJson1, data_socket)

    # 4) Run Spectrum
    comJson1.operationId = 12
    result = sendRequest(comJson1, data_socket)
    print(f"Measurement Done")
    return json.loads(result)


def startDriverProcess():
    with open('driverOutput','w') as f:
        cli_path = os.path.join('.', 'TricordrCLI', 'bin', 'TricordrCLI')
        subprocess.call(cli_path, stdout=f)

def printDebug(debugSocket):
    while 1:
        try:
            str = debugSocket.recv(4096).decode('utf-8')
        except:
            #the socket has been closed
            print('Socket Closed!')
            return

if __name__ == '__main__':
    #start the driver process and wait some time till it's ready
    threading.Thread(target= startDriverProcess).start()
    time.sleep(5)

    #connect using sockets
    data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    data_socket.connect((HOST, DATA_PORT))

    comJson1 = CommunicationJson()
    comJson1.operationId = 0

    # Send connect request and get scanner ID
    response = sendRequest(comJson1,data_socket)
    debug_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    debug_socket.connect((HOST, DEBUG_PORT))
    threading.Thread(target= lambda : printDebug(debug_socket)).start()
    comJson1.scannerId = json.loads(response)['scannerId']
    print(f'Scanner id: {comJson1.scannerId}')

    #Get FW Version Flow
    print('Getting scanner firmware version...')
    comJson1.operationId = 1
    response = sendRequest(comJson1, data_socket)
    print(f'Firmware version: {json.loads(response)["firmwareVersion"]}')

    # Get System Temperature Flow
    print('Getting scanner temperature...')
    comJson1.operationId = 2
    response = sendRequest(comJson1, data_socket)
    print(f'Scanner Temperature: {json.loads(response)["systemTemperatureReading"]}')

    #Run spectrum Flow
    print('Getting scanner spectrum...')
    setSpectrumSettings(comJson1, data_socket)
    runBackground(comJson1, data_socket)
    input()
    result = runMaterial(comJson1, data_socket)
    with open('material_result.json', 'w') as f:
        f.write(result)
    #result = runSpectrumFlow(comJson1, data_socket)
    #settings = result['scanSettings']
    #spectrum = result['runSpectrumResult']
    #print(settings)

    #print(f'Num elements: {len(spectrum)}')

    #plt.subplot(2, 2, 1)
    #plt.plot(spectrum[0], spectrum[1])

    #plt.subplot(2, 2, 2)
    #plt.plot(spectrum[2], spectrum[3])

    #read saved backGround from memory Flow
    print('Reading saved background...')
    comJson1.operationId = 18
    comJson1.scanNum = 0    #0 based index
    response = sendRequest(comJson1, data_socket)
    with open('background_result.json', 'w') as f:
        f.write(response)

    dq = json.loads(response)['retrievedScan']['dq']
    print(dq.keys())

    plt.plot(dq['aJ'], dq['aK'])
    plt.show()

    #Clear Memory Flow
    print('Clearing memory...')
    comJson1.operationId = 16
    response = sendRequest(comJson1, data_socket)
    print(json.loads(response))

    #close and finalize
    print('Closing socket...')
    data_socket.send(b'Close\n')

    #wait some time for the driver process to finalize
    time.sleep(1)
    debug_socket.close()
    data_socket.close()

