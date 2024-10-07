import nano
import time

sensor = nano.Nano()
sensor.connect()

sensor.configure(exposure=10.5, framePeriod=11)

captureResponse = sensor.capture(prefix="test",maxFramesPerCube=2000)
print("Capturing into folder:" + captureResponse["folder"])

for i in range(0,5):
  if not sensor.isCapturing():
    break
  print("Capturing...")
  time.sleep(1)

stoppedCapture = sensor.stopCapture()
if stoppedCapture:
    print("Capture was stopped!\n")

sensor.disconnect()

