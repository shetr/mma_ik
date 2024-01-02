// Fill out your copyright notice in the Description page of Project Settings.


#include "VectorNDim.h"

VectorNDim::VectorNDim() : VectorNDim(1)
{
}

VectorNDim::VectorNDim(int dim)
{
	values.SetNum(dim);
}

VectorNDim::~VectorNDim()
{
}

void VectorNDim::Reset(int dim)
{
	values.SetNum(dim);
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

void VectorNDim::Set(FVector v)
{
	values[0] = v.X;
	values[1] = v.Y;
	values[2] = v.Z;
}
