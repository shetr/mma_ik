// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"

#include "MatrixMxN.h"

/**
 * 
 */
class MMA_IK_API ChainData
{
public:
	ChainData();
	~ChainData();

	float GetSegmentLength() const { return TotalChainLength / NumberOfSegments; }

	float TotalChainLength = 200.0f;
	int32 NumberOfSegments = 10;
	TArray<AStaticMeshActor*> ChildSegments;
	FVector EndEffectorPos;
	TArray<FVector> SegmentAngles;
	TArray<FMatrix> SegmentLocalTransforms;
	TArray<TStaticArray<FMatrix, 3>> SegmentGlobalRotations;
	TArray<FMatrix> SegmentGlobalTransforms;
	MatrixMxN Jacobian;
	AStaticMeshActor* TargetPoint;


	void Reset();
	void RecomputeSegmentTransforms();
	void RecomputeJacobian();
	void TransformSegments(const FVector& origin);
};
