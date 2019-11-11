
import u3
import sys
import time
from time import sleep

def cycle( device, fio, rate, duration ):
    sleepTime = 1.0/rate;
    device.setFIOState(fio,0);
    for i in range(0, int(duration/sleepTime)):
        
        device.setFIOState(fio,1);
        sleep( sleepTime );
        device.setFIOState(fio,0);
        
def multicycle( device, fio1, fio2, rate, duration ):
    sleepTime = 1.0/rate;
    #device.setFIOState(fio,0);
    device.getFeedback(u3.BitStateWrite(fio1, 0),u3.BitStateWrite(fio2, 0),u3.BitStateWrite(6, 0))
    start = time.time()
    current = start
    emgSig = 1
    DAC0_VALUE = d.voltageToDACBits(3.5, dacNumber = 0, is16Bits = False)
    d.getFeedback(u3.DAC0_8(DAC0_VALUE))        # Set DAC0 to 1.5 V
    while( (current - start) < duration):
        print current - start
        #device.setFIOState(fio,1);
        device.getFeedback(u3.BitStateWrite(fio1, 1),u3.BitStateWrite(fio2, 1),u3.BitStateWrite(6, 1) )
	emgSig = 0
        sleep( sleepTime );
        #device.setFIOState(fio,0);
        device.getFeedback(u3.BitStateWrite(fio1, 0),u3.BitStateWrite(fio2, 0),u3.BitStateWrite(6, 0) )
        current = time.time()
    DAC0_VALUE = d.voltageToDACBits(0, dacNumber = 0, is16Bits = False)
    d.getFeedback(u3.DAC0_8(DAC0_VALUE))        # Set DAC0 to 1.5 V
    
		
if __name__ == "__main__":
    numSecondsToRun = float(sys.argv[1])
    print("Opening LabJack...")
    print("Running for " + str(numSecondsToRun) + " seconds.")
    d = u3.U3();
    currentConfig = d.configIO()["FIOAnalog"]
    print(currentConfig)
    # 0xb == 0b00001011  turn 2,4,5,6,7 to digital
    newConfig = currentConfig & 0xb
    d.configIO(FIOAnalog = 11)
    print(newConfig)
    currentConfig = d.configIO()["FIOAnalog"]
    print(currentConfig)
    d.getFeedback(u3.BitDirWrite(4, 1),u3.BitDirWrite(5, 1),u3.BitDirWrite(6, 1))
    print("Running Cycles...")
    multicycle( d,4,5, 5000, numSecondsToRun)
    
    print("Done.")
	
		
