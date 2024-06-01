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

print(f"Subscribed to channel {channel}")

image_count = 0
start_time = time.time()
for message in pubsub.listen():
    if message['type'] == 'message':
        message_data = json.loads(message['data'])
        
        # get image meta
        timestamp = message_data['timestamp']
        frame_id = message_data['frame_id']
        image_bytes_hex = message_data['data']
        timestamp_unix = timestamp['secs'] + timestamp['nsecs'] / 1e9
        
        # get image
        image_bytes = bytes.fromhex(image_bytes_hex)
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image_array = image_array.reshape((1080, 1920, 3))
        
        # get marker
        marker = time.time()
        print(marker - timestamp_unix)
        
        # Check if the image was received within the past 1 second
        if marker - timestamp_unix <= 1:
            image_count += 1
        
        # Check if 1 second has passed
        if time.time() - start_time >= 1:
            break

print(f"Received {image_count} images in the past 1 second")