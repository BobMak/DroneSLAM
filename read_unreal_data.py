from multiprocessing import shared_memory
import numpy as np

memory_name = "droneslam"
# Attach to the existing shared memory block
existing_shm = shared_memory.SharedMemory(name=memory_name)
intro_msg = np.ndarray((13,), dtype=np.uint8, buffer=existing_shm.buf)
print(intro_msg)

while True:
    dt = np.ndarray((1,), dtype=np.float32, buffer=existing_shm.buf[13:13+4])
    print(dt)

# When done, release the shared memory block
existing_shm.close()