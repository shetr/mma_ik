// Fill out your copyright notice in the Description page of Project Settings.


#include "ChainData.h"

ChainData::ChainData()
{
}

ChainData::~ChainData()
{
}

void ChainData::Reset()
{
	this->SegmentAngles.Init(FVector::Zero(), this->NumberOfSegments);
	this->SegmentLocalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	this->SegmentGlobalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	Jacobian.SetNum(3 * NumberOfSegments, 3);

}

void ChainData::RecomputeSegmentTransforms()
{
	FMatrix prevTransform = FMatrix::Identity;
	FMatrix prevRotation = FMatrix::Identity;
	this->SegmentLocalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	this->SegmentGlobalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	this->SegmentGlobalRotations.Init(TStaticArray<FMatrix, 3>(), this->NumberOfSegments);
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		// Rotates around X in CW order (when X is pointing towards us)
		FMatrix xRot = FMatrix::Identity;
		xRot.M[1][1] =  cos(this->SegmentAngles[i].X); xRot.M[1][2] = -sin(this->SegmentAngles[i].X);
		xRot.M[2][1] =  sin(this->SegmentAngles[i].X); xRot.M[2][2] =  cos(this->SegmentAngles[i].X);
		// Rotates around Y in CW order (when Y is pointing towards us)
		FMatrix yRot = FMatrix::Identity;
		yRot.M[0][0] =  cos(this->SegmentAngles[i].Y); yRot.M[0][2] =  sin(this->SegmentAngles[i].Y);
		yRot.M[2][0] = -sin(this->SegmentAngles[i].Y); yRot.M[2][2] =  cos(this->SegmentAngles[i].Y);
		// Rotates around Z in CW order (when Z is pointing towards us)
		FMatrix zRot = FMatrix::Identity;
		zRot.M[0][0] =  cos(this->SegmentAngles[i].Z); zRot.M[0][1] = -sin(this->SegmentAngles[i].Z);
		zRot.M[1][0] =  sin(this->SegmentAngles[i].Z); zRot.M[1][1] =  cos(this->SegmentAngles[i].Z);

		this->SegmentGlobalRotations[i][2] = prevRotation;
		this->SegmentGlobalRotations[i][1] = this->SegmentGlobalRotations[i][2] * zRot;
		this->SegmentGlobalRotations[i][0] = this->SegmentGlobalRotations[i][1] * yRot;
		prevRotation = this->SegmentGlobalRotations[i][0] * xRot;

		FMatrix localRotation = zRot * yRot * xRot;
		FMatrix localTranslation = FMatrix::Identity;
		localTranslation.SetColumn(3, FVector(0, 0, this->GetSegmentLength()));
		FMatrix localTransform = localRotation * localTranslation;
		this->SegmentLocalTransforms[i] = localTransform;
		FMatrix globalTransform = prevTransform * localTransform;
		this->SegmentGlobalTransforms[i] = globalTransform;
		prevTransform = globalTransform;
	}
}

FVector MyMatMul(const FMatrix& m, const FVector& v)
{
	FVector x;
	x.X = m.M[0][0] * v.X + m.M[0][1] * v.Y + m.M[0][2] * v.Z;
	x.Y = m.M[1][0] * v.X + m.M[1][1] * v.Y + m.M[1][2] * v.Z;
	x.Z = m.M[2][0] * v.X + m.M[2][1] * v.Y + m.M[2][2] * v.Z;
	return x;
}

void ChainData::RecomputeJacobian()
{
	Jacobian.SetNum(3 * NumberOfSegments, 3);
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		FVector segmentLocation = ChildSegments[i]->GetActorLocation();
		FVector diff = EndEffectorPos - segmentLocation;

		FVector xAxis = MyMatMul(this->SegmentGlobalRotations[i][0], FVector::XAxisVector);
		FVector yAxis = MyMatMul(this->SegmentGlobalRotations[i][1], FVector::YAxisVector);
		FVector zAxis = MyMatMul(this->SegmentGlobalRotations[i][2], FVector::ZAxisVector);

		// TODO: IF BUG CORRECT pitch and roll axis 
		FVector xRotDir = FVector::CrossProduct(xAxis, diff);
		FVector yRotDir = FVector::CrossProduct(yAxis, diff);
		FVector zRotDir = FVector::CrossProduct(zAxis, diff);
		Jacobian(3 * i + 0, 0) = xRotDir.X;
		Jacobian(3 * i + 0, 1) = xRotDir.Y;
		Jacobian(3 * i + 0, 2) = xRotDir.Z;

		Jacobian(3 * i + 1, 0) = yRotDir.X;
		Jacobian(3 * i + 1, 1) = yRotDir.Y;
		Jacobian(3 * i + 1, 2) = yRotDir.Z;

		Jacobian(3 * i + 2, 0) = zRotDir.X;
		Jacobian(3 * i + 2, 1) = zRotDir.Y;
		Jacobian(3 * i + 2, 2) = zRotDir.Z;
		#if 0
		UE_LOG(LogTemp, Warning, TEXT("diff %d: %f, %f, %f"), i, diff.X, diff.Y, diff.Z);
		#endif
		#if 0
		UE_LOG(LogTemp, Warning, TEXT("xAxis %d: %f, %f, %f, l: %f"), i, xAxis.X, xAxis.Y, xAxis.Z, xAxis.Length());
		UE_LOG(LogTemp, Warning, TEXT("yAxis %d: %f, %f, %f, l: %f"), i, yAxis.X, yAxis.Y, yAxis.Z, yAxis.Length());
		UE_LOG(LogTemp, Warning, TEXT("zAxis %d: %f, %f, %f, l: %f"), i, zAxis.X, zAxis.Y, zAxis.Z, zAxis.Length());
		#endif
		#if 0
		UE_LOG(LogTemp, Warning, TEXT("J%d X: %f, %f, %f"), i, Jacobian(3 * i + 0, 0), Jacobian(3 * i + 0, 1), Jacobian(3 * i + 0, 2));
		UE_LOG(LogTemp, Warning, TEXT("J%d Y: %f, %f, %f"), i, Jacobian(3 * i + 1, 0), Jacobian(3 * i + 1, 1), Jacobian(3 * i + 1, 2));
		UE_LOG(LogTemp, Warning, TEXT("J%d Z: %f, %f, %f"), i, Jacobian(3 * i + 2, 0), Jacobian(3 * i + 2, 1), Jacobian(3 * i + 2, 2));
		#endif
	}
}

void ChainData::TransformSegments(const FVector& origin)
{
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		FVector posStart = FVector::Zero();
		if (i > 0) {
			posStart = this->SegmentGlobalTransforms[i - 1].GetColumn(3);
		}
		FVector posEnd = this->SegmentGlobalTransforms[i].GetColumn(3);
		//UE_LOG(LogTemp, Warning, TEXT("i: %d, posStart: %f, %f, %f"), i, posStart.X, posStart.Y, posStart.Z);
		//UE_LOG(LogTemp, Warning, TEXT("i: %d, posEnd:   %f, %f, %f"), i, posEnd.X, posEnd.Y, posEnd.Z);

		this->ChildSegments[i]->SetActorLocation(origin + posStart);

		FVector dir = (posEnd - posStart).GetSafeNormal();
		FQuat rot = FQuat::FindBetween(FVector::UpVector, dir);
		this->ChildSegments[i]->SetActorRotation(rot);
	}
	EndEffectorPos = origin + this->SegmentGlobalTransforms.Last().GetColumn(3);
}
