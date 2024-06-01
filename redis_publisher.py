import time
import redis
import core

# 设置
frequency = 30.0
interval = 1.0 / frequency
channel = 'camera'

def main():
    frequency = 30.0

    r = redis.StrictRedis(host='localhost', port=6379, db=0)

    frame_id = 0
    log_count = 0
    log_last_time = time.time()
    next_time = time.perf_counter() + interval

    try:
        while True:
            current_time = time.perf_counter()
            if current_time < next_time:
                time.sleep(next_time - current_time)
            next_time += 1/frequency
            
            image = core.generate_random_image(1920, 1080)
            timestamp = core.generate_timestamp()
            
            # create data
            frame_id_bytes = frame_id.to_bytes(4, byteorder='big')
            ts_secs = timestamp[0].to_bytes(4, byteorder='big')
            ts_nsecs = timestamp[1].to_bytes(4, byteorder='big')
            
            data = frame_id_bytes + ts_secs + ts_nsecs + image.tobytes()
            
            r.publish(channel, data)
            frame_id += 1
            log_count += 1

            # print log
            if time.time() - log_last_time >= 1:
                print(f"Published {log_count} frames in the last second")
                log_last_time = time.time()
                log_count = 0

    except KeyboardInterrupt:
        print("Goodbye!")
    finally:
        r.close()

if __name__ == '__main__':
    main()