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
	MatrixMxN();
	MatrixMxN(int width, int height);
	~MatrixMxN();

	void SetNum(int width, int height);

	int GetWidth() const { return _width; }
	int GetHeight() const { return _height; }

	double operator ()(int i, int j) const;
	double& operator ()(int i, int j);

	void Clone(const MatrixMxN& m);
	void ClonePart(const MatrixMxN& m);

	void Multiply(const VectorNDim& in, VectorNDim& out) const;

	void Multiply(const MatrixMxN& right, MatrixMxN& out) const;

	double Determinant3x3() const;

	void Inverse3x3(MatrixMxN& out) const;

	void Inverse(MatrixMxN& out) const;

	bool OnlyZeros() const;

	// delete this, moved to jacobian inverse solver
	void PseoudoInverse3x3(MatrixMxN& out) const;

	void Transpose(MatrixMxN& out);

	void SwapRows(int i, int j);
private:
	int _width;
	int _height;
	TArray<double> _values;
};
