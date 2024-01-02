// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IK_SolverBase.h"
#include "JacobianTransposeIK_Solver.generated.h"

/**
 * 
 */
UCLASS()
class MMA_IK_API AJacobianTransposeIK_Solver : public AIK_SolverBase
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AJacobianTransposeIK_Solver();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void Reset(ChainData& data) override;

	virtual void Solve(ChainData& data, const FVector& origin, float DeltaTime) override;

private:
	MatrixMxN JacobianTranspose;
	VectorNDim dX;
	VectorNDim dO;
};
