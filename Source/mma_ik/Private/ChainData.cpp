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

		// TODO: IF BUG CORRECT pitch and roll axis 
		FVector pitchDir = FVector::CrossProduct(this->SegmentGlobalTransforms[i].TransformVector(FVector::YAxisVector), diff);
		FVector rollDir = FVector::CrossProduct(this->SegmentGlobalTransforms[i].TransformVector(FVector::XAxisVector), diff);
		FVector yawDir = FVector::CrossProduct(this->SegmentGlobalTransforms[i].TransformVector(FVector::ZAxisVector), diff);
		Jacobian(3 * i + 0, 0) = pitchDir.X;
		Jacobian(3 * i + 0, 1) = pitchDir.Y;
		Jacobian(3 * i + 0, 2) = pitchDir.Z;

		Jacobian(3 * i + 1, 0) = rollDir.X;
		Jacobian(3 * i + 1, 1) = rollDir.Y;
		Jacobian(3 * i + 1, 2) = rollDir.Z;

		//Jacobian(3 * i + 2, 0) = yawDir.X;
		//Jacobian(3 * i + 2, 1) = yawDir.Y;
		//Jacobian(3 * i + 2, 2) = yawDir.Z;
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
