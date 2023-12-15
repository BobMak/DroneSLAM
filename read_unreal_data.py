from multiprocessing import shared_memory
import numpy as np

memory_name = "droneslam"
# Attach to the existing shared memory block
existing_shm = shared_memory.SharedMemory(name=memory_name)

# Note: Replace "shm_block_name_here" with the actual name printed by the first process.

intro_msg = np.ndarray((13,), dtype=np.uint8, buffer=existing_shm.buf)
print(intro_msg)

IDX_POS = 14
IDX_ORI = 26
IDX_CAM = 38
RES_CAM = (3,256,256)
tot_cam = np.prod(RES_CAM)

while True:
    dt = np.ndarray((1,), dtype=np.float32, buffer=existing_shm.buf[13:13+4])
    # pos = np.ndarray((3,), dtype=np.float32, buffer=existing_shm.buf[IDX_POS:IDX_POS+12])
    # ori = np.ndarray((3,), dtype=np.float32, buffer=existing_shm.buf[IDX_ORI:IDX_ORI+12])
    # cam = np.ndarray((3,256,256), dtype=np.float32, buffer=existing_shm.buf[IDX_CAM:IDX_CAM+tot_cam])
    print(dt)

# When done, release the shared memory block
existing_shm.close()