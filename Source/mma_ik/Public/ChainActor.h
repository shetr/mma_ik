// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ChainActor.generated.h"

UCLASS()
class MMA_IK_API AChainActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AChainActor();

private:
	UStaticMeshComponent* ChainMesh;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	TArray<AChainActor*> childChains;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void CreateChildCube(const FVector& RelativeLocation);

};
