// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IK_SolverBase.h"
#include "JacobianInverseIK_Solver.generated.h"

/**
 * 
 */
UCLASS()
class MMA_IK_API AJacobianInverseIK_Solver : public AIK_SolverBase
{
	GENERATED_BODY()
public:
	// Sets default values for this actor's properties
	AJacobianInverseIK_Solver();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void Reset(ChainData& data) override;

	virtual void Solve(ChainData& data, const FVector& origin, float DeltaTime) override;

private:
	void ReduceJacobian(const MatrixMxN& J);

	// Expects J to be matrix with 3 height and rank smaller than 3
	void GetPseudoinv(const MatrixMxN& J, const MatrixMxN& JT);

private:
	MatrixMxN JJT;
	MatrixMxN JTJ;
	MatrixMxN JJTinv;
	MatrixMxN Jpinv;
	MatrixMxN JacobianTranspose;
	VectorNDim dX;
	VectorNDim dO;

	MatrixMxN JS;
	MatrixMxN JV;
	MatrixMxN JVT;
	MatrixMxN JU;


	MatrixMxN Temp;

	MatrixMxN RJ;
	VectorNDim RdX;


	MatrixMxN CJ;
	VectorNDim CdX;
};
