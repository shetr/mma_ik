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

double MatrixMxN::operator()(int i, int j) const
{
	return _values[i + j * _width];
}

double& MatrixMxN::operator()(int i, int j)
{
	return _values[i + j * _width];
}

void MatrixMxN::Clone(const MatrixMxN& m)
{
	SetNum(m.GetWidth(), m.GetHeight());
	for (int h = 0; h < m.GetHeight(); ++h) {
		for (int w = 0; w < m.GetWidth(); ++w) {
			(*this)(w, h) = m(w, h);
		}
	}
}

void MatrixMxN::ClonePart(const MatrixMxN& m)
{
	for (int h = 0; h < GetHeight() && h < m.GetHeight(); ++h) {
		for (int w = 0; w < GetWidth() && w < m.GetWidth(); ++w) {
			(*this)(w, h) = m(w, h);
		}
	}
}

void MatrixMxN::Multiply(const VectorNDim& in, VectorNDim& out) const
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

void MatrixMxN::Multiply(const MatrixMxN& right, MatrixMxN& out) const
{
	out.SetNum(right.GetWidth(), _height);
	check(_width == right.GetHeight());
	for (size_t k = 0; k < right.GetWidth(); k++)
	{
		for (size_t j = 0; j < _height; j++)
		{
			out(k, j) = 0;
			for (size_t i = 0; i < _width; i++)
			{
				out(k, j) += (*this)(i, j) * right(k, i);
			}
		}
	}
}

double MatrixMxN::Determinant3x3() const
{
	check(_width == 3);
	check(_height == 3);
	const MatrixMxN& m = *this;
	double det = m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) -
		m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
		m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
	return det;
}

void MatrixMxN::Inverse3x3(MatrixMxN& out) const
{
	check(_width == 3);
	check(_height == 3);
	out.SetNum(3, 3);

	double invdet = 1 / Determinant3x3();

	const MatrixMxN& m = *this;
	out(0, 0) = (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * invdet;
	out(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * invdet;
	out(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * invdet;
	out(1, 0) = (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * invdet;
	out(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * invdet;
	out(1, 2) = (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2)) * invdet;
	out(2, 0) = (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1)) * invdet;
	out(2, 1) = (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1)) * invdet;
	out(2, 2) = (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1)) * invdet;
}

void MatrixMxN::Inverse(MatrixMxN& out) const
{

	check(_width <= 3);
	check(_height <= 3);
	check(_width == _height);

	const MatrixMxN& m = *this;
	if (_width == 1) {
		out.SetNum(1, 1);
		out(0, 0) = 1.0 / m(0, 0);
	}
	else if (_width == 2) {
		out.SetNum(2, 2);
		double det = m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
		double invdet = 1 / det;
		out(0, 0) =  invdet * m(1, 1);
		out(1, 0) = -invdet * m(0, 1);
		out(0, 1) = -invdet * m(1, 0);
		out(1, 1) =  invdet * m(0, 0);
	}
	else if (_width == 3) {
		Inverse3x3(out);
	}
}

bool MatrixMxN::OnlyZeros() const
{
	for (int i = 0; i < _values.Num(); ++i) {
		if (_values[i] != 0.0) {
			return false;
		}
	}
	return true;
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

void MatrixMxN::SwapRows(int i, int j)
{
	check(i < GetHeight());
	check(j < GetHeight());
	for (int w = 0; w < GetWidth(); ++w) {
		Swap((*this)(w, i), (*this)(w, j));
	}
}
