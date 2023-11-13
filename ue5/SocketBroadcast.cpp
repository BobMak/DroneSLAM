// Fill out your copyright notice in the Description page of Project Settings.

#include <iostream>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cerrno>

#include "SocketBroadcast.h"

// Sets default values for this component's properties
USocketBroadcast::USocketBroadcast()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void USocketBroadcast::BeginPlay()
{
	Super::BeginPlay();

	const char *memname = "droneslam";
    const size_t size = 262182; // Shared memory size, 262144+38
    OutPixels.reserve(262144);

    // Create shared memory object
    int shm_fd = shm_open(memname, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "shm_open error: " << std::strerror(errno) << std::endl;
		UE_LOG(LogTemp, Warning, TEXT( "shm_open error"));
    }

    // Configure the size of the shared memory object
    if (ftruncate(shm_fd, size) == -1) {
        std::cerr << "ftruncate error: " << std::strerror(errno) << std::endl;
		UE_LOG(LogTemp, Warning, TEXT( "ftruncate error"));
    }

    // Map the shared memory
    shared_memory = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_memory == MAP_FAILED) {
        std::cerr << "mmap error: " << std::strerror(errno) << std::endl;
		UE_LOG(LogTemp, Warning, TEXT( "mmap error" ));
    }

    // Use the shared memory
    memcpy(shared_memory, "Hello, World!", 13); // Write to shared memory
    std::cout << static_cast<char*>(shared_memory) << std::endl; // Read from shared memory
	UE_LOG(LogTemp, Warning, TEXT( "posted hello world to the memory"));

}


// Called every frame
void USocketBroadcast::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// share current delta time
	char* write_ptr = static_cast<char*>(shared_memory) + 13;
	memcpy(write_ptr, &DeltaTime, 4);
	// shm_unlink(memname);
}

void USocketBroadcast::SendEgo(UTextureRenderTarget2D* RenderTarget)
{
    // char* write_ptr = static_cast<char*>(shared_memory) + 14;
	// memcpy(write_ptr, &position, 12);
    // write_ptr = static_cast<char*>(shared_memory) + 26;
    // memcpy(write_ptr, &orientation, 12);

    // make an outupt array
    // TArray<FColor> &OutPixels()[256];
    if (!RenderTarget)
        return;
    FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    if (RenderTargetResource)
    {
        FReadSurfaceDataFlags ReadSurfaceDataFlags;
        ReadSurfaceDataFlags.SetLinearToGamma(false); // Depending on your needs

        RenderTargetResource->ReadPixels(OutPixels, ReadSurfaceDataFlags);
    }
    char* write_ptr = static_cast<char*>(shared_memory) + 38;
    memcpy(write_ptr, &OutPixels, 262144);  // 3*256*256
}