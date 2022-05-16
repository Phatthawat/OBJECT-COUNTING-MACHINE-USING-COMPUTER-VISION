
import psutil
#from gpiozero import CPUTemperature

def getCPU():
    return psutil.cpu_percent(interval=1)

def getRAM():
    return psutil.virtual_memory()

def getTemp():
    pass
    #cpu = CPUTemperature()
    #return round(cpu.temperature,1)


