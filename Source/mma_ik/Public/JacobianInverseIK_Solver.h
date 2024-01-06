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

	virtual void Solve(ChainData& data, const FVector& origin, float DeltaTime, bool CCDTopDown) override;

private:
	// If the jacobian equations have rank lower than 3, then this method will reduce it to smaller number of equatins, so that the rank is equal to the number of rows
	void ReduceJacobian(const MatrixMxN& J);

private:
	// J * J^T
	MatrixMxN JJT;
	// J^T * J
	MatrixMxN JTJ;
	// (J * J^T)^-1
	MatrixMxN JJTinv;
	// jacobian pseudoinverse
	MatrixMxN Jpinv;
	// transposed jacobian
	MatrixMxN JT;
	// Vector pointing from the end effector to the target possition
	VectorNDim dX;
	// Angle parameters change
	VectorNDim dO;

	// equation J dO = dX is eliminated to equivalent GJ dO = GdX
	MatrixMxN GJ;
	VectorNDim GdX;

	// RJ dO = RdX is GJ dO = GdX with zero rows droped
	MatrixMxN RJ;
	VectorNDim RdX;
};
