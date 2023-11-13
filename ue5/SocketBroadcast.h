// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SocketBroadcast.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class DRONESLAM_API USocketBroadcast : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	USocketBroadcast();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	// our data sending function
	UFUNCTION(BlueprintCallable, Category="IPC")
	void SendEgo(UTextureRenderTarget2D* RenderTarget);

private:
	void *shared_memory;
	TArray<FColor> OutPixels;
};
