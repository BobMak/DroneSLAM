#include <iostream>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cerrno>

void* shared_memory;

int main()
{
	const char *memname = "/droneslam";
    const size_t size = 1024; // Shared memory size

    // Create shared memory object
    int shm_fd = shm_open(memname, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "shm_open error: " << std::strerror(errno) << std::endl;
        return 1;
    }

    // Configure the size of the shared memory object
    if (ftruncate(shm_fd, size) == -1) {
        std::cerr << "ftruncate error: " << std::strerror(errno) << std::endl;
        return 1;
    }

    // Map the shared memory
    shared_memory = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_memory == MAP_FAILED) {
        std::cerr << "mmap error: " << std::strerror(errno) << std::endl;
        return 1;
    }

    // Use the shared memory
    memcpy(shared_memory, "Hello, World!", 13); // Write to shared memory
    std::cout << static_cast<char*>(shared_memory) << std::endl; // Read from shared memory
	float DeltaTime = 0.0f;
    while(true) {
        sleep(1);
        // share current delta time
        char* write_ptr = static_cast<char*>(shared_memory) + 13;
        memcpy(write_ptr, &DeltaTime, 4);
        DeltaTime += 1.0f;
    }
    return 0;
}
