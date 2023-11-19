// Fill out your copyright notice in the Description page of Project Settings.


#include "VectorNDim.h"

VectorNDim::VectorNDim(int dim)
{
	values.AddZeroed(dim);
}

VectorNDim::~VectorNDim()
{
}

VectorNDim& VectorNDim::operator+=(const VectorNDim& other)
{
	for (size_t i = 0; i < values.Num(); i++)
	{
		values[i] += other.values[i];
	}
	return *this;
}

float VectorNDim::operator[](int i) const
{
	return values[i];
}

float& VectorNDim::operator[](int i)
{
	return values[i];
}
