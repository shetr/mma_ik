// Fill out your copyright notice in the Description page of Project Settings.


#include "MatrixMxN.h"

MatrixMxN::MatrixMxN()
	: MatrixMxN(1, 1)
{
}

MatrixMxN::MatrixMxN(int width, int height)
	: _width(width), _height(height)
{
	_values.SetNum(width * height);
}

MatrixMxN::~MatrixMxN()
{
}

void MatrixMxN::SetNum(int width, int height)
{
	_width = width;
	_height = height;
	_values.Init(0.0f, width * height);
}

float MatrixMxN::operator()(int i, int j) const
{
	return _values[i + j * _width];
}

float& MatrixMxN::operator()(int i, int j)
{
	return _values[i + j * _width];
}

void MatrixMxN::Multiply(const VectorNDim& in, VectorNDim& out)
{
	out.Reset(_height);
	for (size_t j = 0; j < out.GetSize(); j++)
	{
		out[j] = 0;
		for (size_t i = 0; i < in.GetSize(); i++)
		{
			out[j] += (*this)(i, j) * in[i];
		}
	}
}

void MatrixMxN::Transpose(MatrixMxN& out)
{
	out.SetNum(_height, _width);
	for (size_t j = 0; j < _height; j++)
	{
		for (size_t i = 0; i < _width; i++)
		{
			out(j, i) = (*this)(i, j);
		}
	}
}
