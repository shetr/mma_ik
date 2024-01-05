// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
class MMA_IK_API VectorNDim
{
public:
	VectorNDim();
	VectorNDim(int dim);
	~VectorNDim();

	void Clone(const VectorNDim& v);
	void ClonePart(const VectorNDim& v);

	void Reset(int dim);

	void SwapElems(int i, int j);

	VectorNDim& operator +=(const VectorNDim& other);
	double operator[](int i) const;
	double& operator[](int i);

	void Set(FVector v);

	int GetSize() const { return values.Num(); }
private:
	TArray<double> values;
};
