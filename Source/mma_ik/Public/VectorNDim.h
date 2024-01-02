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

	void Reset(int dim);

	VectorNDim& operator +=(const VectorNDim& other);
	float operator[](int i) const;
	float& operator[](int i);

	void Set(FVector v);

	int GetSize() const { return values.Num(); }
private:
	TArray<float> values;
};
