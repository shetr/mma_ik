// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IK_SolverBase.h"
#include "ConstrainedIK_Solver.generated.h"

/**
 * 
 */
UCLASS()
class MMA_IK_API AConstrainedIK_Solver : public AIK_SolverBase
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AConstrainedIK_Solver();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void Reset(ChainData& data) override;

	virtual void Solve(ChainData& data) override;
};
