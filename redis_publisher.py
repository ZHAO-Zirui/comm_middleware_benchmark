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
        
        # create image message
        frame_id_bytes = frame_id.to_bytes(4, byteorder='big')
        ts_secs = timestamp[0].to_bytes(4, byteorder='big')
        ts_nsecs = timestamp[1].to_bytes(4, byteorder='big')
        
        data = frame_id_bytes + ts_secs + ts_nsecs + image.tobytes()
        
        r.publish('camera', data)
        frame_id += 1
        time_last_frame = time.time()

        print(f"Published frame {frame_id}")
except KeyboardInterrupt:
    print("Goodbye!")
finally:
    r.close()