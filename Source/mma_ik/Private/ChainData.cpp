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
	this->SegmentAngles.Init(FRotator::ZeroRotator, this->NumberOfSegments);
	this->SegmentLocalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	this->SegmentGlobalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	Jacobian.SetNum(3 * NumberOfSegments, 3);

}

void ChainData::RecomputeSegmentTransforms()
{
	FMatrix prevTransform = FMatrix::Identity;
	this->SegmentLocalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	this->SegmentGlobalTransforms.Init(FMatrix::Identity, this->NumberOfSegments);
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		FMatrix localRotation = FRotationMatrix::Make(this->SegmentAngles[i]);
		FMatrix localTranslation = FMatrix::Identity;
		localTranslation.SetColumn(3, FVector(0, 0, this->GetSegmentLength()));
		FMatrix localTransform = localRotation * localTranslation;
		this->SegmentLocalTransforms[i] = localTransform;
		FMatrix globalTransform = prevTransform * localTransform;
		this->SegmentGlobalTransforms[i] = globalTransform;
		prevTransform = globalTransform;
	}
}

void ChainData::RecomputeJacobian()
{
	Jacobian.SetNum(3 * NumberOfSegments, 3);
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		FVector segmentLocation = ChildSegments[i]->GetActorLocation();
		FVector diff = EndEffectorPos - segmentLocation;

		FVector xAxis = this->SegmentGlobalTransforms[i].TransformVector(FVector::XAxisVector);
		FVector yAxis = this->SegmentGlobalTransforms[i].TransformVector(FVector::YAxisVector);
		FVector zAxis = this->SegmentGlobalTransforms[i].TransformVector(FVector::ZAxisVector);

		// TODO: IF BUG CORRECT pitch and roll axis 
		FVector pitchDir = FVector::CrossProduct(yAxis, diff);
		FVector rollDir = FVector::CrossProduct(xAxis, diff);
		FVector yawDir = FVector::CrossProduct(zAxis, diff);
		Jacobian(3 * i + 0, 0) = pitchDir.X;
		Jacobian(3 * i + 0, 1) = pitchDir.Y;
		Jacobian(3 * i + 0, 2) = pitchDir.Z;

		Jacobian(3 * i + 1, 0) = rollDir.X;
		Jacobian(3 * i + 1, 1) = rollDir.Y;
		Jacobian(3 * i + 1, 2) = rollDir.Z;

		//Jacobian(3 * i + 2, 0) = yawDir.X;
		//Jacobian(3 * i + 2, 1) = yawDir.Y;
		//Jacobian(3 * i + 2, 2) = yawDir.Z;
		#if 0
		UE_LOG(LogTemp, Warning, TEXT("diff %d: %f, %f, %f"), i, diff.X, diff.Y, diff.Z);
		#endif
		#if 1
		UE_LOG(LogTemp, Warning, TEXT("xAxis %d: %f, %f, %f, l: %f"), i, xAxis.X, xAxis.Y, xAxis.Z, xAxis.Length());
		UE_LOG(LogTemp, Warning, TEXT("yAxis %d: %f, %f, %f, l: %f"), i, yAxis.X, yAxis.Y, yAxis.Z, yAxis.Length());
		UE_LOG(LogTemp, Warning, TEXT("zAxis %d: %f, %f, %f, l: %f"), i, zAxis.X, zAxis.Y, zAxis.Z, zAxis.Length());
		#endif
		#if 1
		UE_LOG(LogTemp, Warning, TEXT("J%d pitch: %f, %f, %f"), i, Jacobian(3 * i + 0, 0), Jacobian(3 * i + 0, 1), Jacobian(3 * i + 0, 2));
		UE_LOG(LogTemp, Warning, TEXT("J%d  roll: %f, %f, %f"), i, Jacobian(3 * i + 1, 0), Jacobian(3 * i + 1, 1), Jacobian(3 * i + 1, 2));
		UE_LOG(LogTemp, Warning, TEXT("J%d   yaw: %f, %f, %f"), i, Jacobian(3 * i + 2, 0), Jacobian(3 * i + 2, 1), Jacobian(3 * i + 2, 2));
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
