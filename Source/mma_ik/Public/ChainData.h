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

	// total chain length
	float TotalChainLength = 200.0f;
	// number of segments
	int32 NumberOfSegments = 10;
	// child segments actors
	TArray<AStaticMeshActor*> ChildSegments;
	// Recomputed end effector position
	FVector EndEffectorPos;
	// Euler angles parameters for each segment
	TArray<FVector> SegmentAngles;
	// Transform of each segment in chain to the previous one (currently not used)
	TArray<FMatrix> SegmentLocalTransforms;
	// Euler angles rotations for each segment
	TArray<TStaticArray<FMatrix, 3>> SegmentGlobalRotations;
	// Global transforms of each segment with origin positioned at chain start
	TArray<FMatrix> SegmentGlobalTransforms;
	// Jacobian - change of the end effector position with respect to segment euler angles
	MatrixMxN Jacobian;
	// Target point
	AStaticMeshActor* TargetPoint;


	void Reset();
	// Recomputes segment transforms from current angle parameters
	void RecomputeSegmentTransforms();
	// Recomputes jacobian from current angles and positions
	void RecomputeJacobian();
	// Transforms segmets with current transform matricies
	void TransformSegments(const FVector& origin);
};
