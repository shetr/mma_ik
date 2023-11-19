// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "VectorNDim.h"

#include "CoreMinimal.h"

/**
 * 
 */
class MMA_IK_API MatrixMxN
{
public:
	MatrixMxN(int width, int height);
	~MatrixMxN();

	int GetWidth() const { return _width; }
	int GetHeight() const { return _height; }

	float operator ()(int i, int j) const;
	float& operator ()(int i, int j);

	void Multiply(const VectorNDim& in, VectorNDim& out);

	void Transpose(MatrixMxN& out);
private:
	int _width;
	int _height;
	TArray<float> _values;
};
