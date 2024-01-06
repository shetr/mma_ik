// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ChainData.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "IK_SolverBase.generated.h"

UCLASS()
class MMA_IK_API AIK_SolverBase : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AIK_SolverBase();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void Reset(ChainData& data);

	virtual void Solve(ChainData& data, const FVector& origin, float DeltaTime, bool CCDTopDown);
};
