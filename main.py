import dotenv 
import os 
from sendubidotsw10 import sendData
from week10 import averageOksi
oksi = averageOksi()
sendData(oksi[0],oksi[1])
