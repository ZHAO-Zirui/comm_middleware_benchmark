import json
import time
import redis
import numpy as np
import core

# Settings
channel = 'camera'

client = redis.StrictRedis(host='localhost', port=6379, db=0)

pubsub = client.pubsub()
pubsub.subscribe(channel)

for message in pubsub.listen():
    if message['type'] == 'message':
        data = message['data']
        
        # get image meta
        frame_id = int.from_bytes(data[:4], byteorder='big')
        ts_secs = int.from_bytes(data[4:8], byteorder='big')
        ts_nsecs = int.from_bytes(data[8:12], byteorder='big')
        timestamp_unix = ts_secs + ts_nsecs / 1e9
        
        # get image
        image_bytes = data[12:]
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image_array = image_array.reshape((1080, 1920, 3))
        
        # get marker
        marker = time.time()
        print(marker - timestamp_unix)
