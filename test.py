import time
from datetime import datetime

delay = 0.1

timestamp = time.time()

timestamp = timestamp - delay
print(datetime.fromtimestamp(timestamp).strftime("%H:%M:%S:%f") )