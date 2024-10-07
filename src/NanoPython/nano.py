#!/usr/bin/env python

import sys
import telnetlib
import time
import json

class Nano:
    def __init__(self,host="10.0.65.50", port=8100, verbose=False):
        self.host = host
        self.port = port
        self.verbose = verbose
    
    def connect(self):
        print("Connecting to " + self.host + "@" + str(self.port) + "\n")
        self.telnetObject = telnetlib.Telnet(self.host, self.port)
        self.telnetObject.read_until(b"hpi> ")	
        

    def disconnect(self):
        print('Disconnecting')
        self.telnetObject.write(b"exit\r")
        self.telnetObject.read_all()
        return

    def sendCommand(self,cmd):
        self.telnetObject.write(str.encode(cmd) + b"\r")
        self.telnetObject.read_until(b"\r\n")
        response = bytes.decode(self.telnetObject.read_until(b"\r"))
        resp = json.loads(response)
        self.telnetObject.read_until(b"hpi> ")
        if self.verbose:
            print(resp)
            sys.stdout.flush()
        if not resp["success"]:
            print("Error! " + resp["errorMessage"])
            self.disconnect() # self.telnetObject)
            sys.exit(1)
        return resp["response"]

    def configure(self,exposure,framePeriod):
        jsonObject = {"exposure": exposure, "framePeriod": framePeriod}
        jsonText = json.dumps(jsonObject)
        return self.sendCommand("configure(" + jsonText +")")

    def capture(self,prefix="",maxCubes=0,maxFramesPerCube=0):
        jsonObject = {"prefix":prefix, "maxCubes": maxCubes, "maxFramesPerCube": maxFramesPerCube}
        jsonText = json.dumps(jsonObject)
        return self.sendCommand("capture(" + jsonText +")")

    def isCapturing(self):
        return self.sendCommand("isCapturing")

    def stopCapture(self):
        return self.sendCommand("stopCapture")
        
if __name__ == "__main__":
    print("This is a module.  Try again.")
    sys.exit(2)
