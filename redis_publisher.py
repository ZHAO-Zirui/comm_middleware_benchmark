import json
import time
import redis
import core

# Settings
frequency = 30
channel = 'camera'

r = redis.StrictRedis(host='localhost', port=6379, db=0)

frame_id = 0
time_last_frame = time.time()
try:
    while True:
        if time.time() - time_last_frame < (1 / frequency):
            continue
        
        image = core.generate_random_image(1920, 1080)
        timestamp = core.generate_timestamp()
        
        image_hex = image.tobytes().hex() # 0.03s
        
        message = {
            'type': 'image',
            'frame_id': frame_id,
            'timestamp': {
                'secs': timestamp[0],
                'nsecs': timestamp[1]
            },
            'data': image_hex
        }
        
        message_json = json.dumps(message)  # 0.02s
        
        r.publish('camera', message_json)
        frame_id += 1
        time_last_frame = time.time()

        print(f"Published frame {frame_id}")
except KeyboardInterrupt:
    print("Goodbye!")
finally:
    r.close()