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
		localTranslation.M[2][3] = this->GetSegmentLength();
		FMatrix localTransform = localTranslation;
		localTransform *= localRotation;
		this->SegmentLocalTransforms[i] = localTransform;
		FMatrix globalTransform = localTransform;
		globalTransform *= prevTransform;
		this->SegmentGlobalTransforms[i] = globalTransform;
		prevTransform = globalTransform;
	}
}

void ChainData::RecomputeJacobian(FVector target)
{
	Jacobian.SetNum(3 * NumberOfSegments, 3);
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		FVector segmentLocation = ChildSegments[i]->GetActorLocation();
		FVector diff = target - segmentLocation;

		// TODO: IF BUG CORRECT pitch and roll axis 
		FVector pitchAxis = FVector::CrossProduct(this->SegmentGlobalTransforms[i].TransformVector(FVector::YAxisVector), diff);
		FVector rollAxis = FVector::CrossProduct(this->SegmentGlobalTransforms[i].TransformVector(FVector::XAxisVector), diff);
		FVector yawAxis = FVector::CrossProduct(this->SegmentGlobalTransforms[i].TransformVector(FVector::ZAxisVector), diff);
		Jacobian(3 * i + 0, 0) = pitchAxis.X;
		Jacobian(3 * i + 0, 1) = pitchAxis.Y;
		Jacobian(3 * i + 0, 2) = pitchAxis.Z;

		Jacobian(3 * i + 1, 0) = rollAxis.X;
		Jacobian(3 * i + 1, 1) = rollAxis.Y;
		Jacobian(3 * i + 1, 2) = rollAxis.Z;

		Jacobian(3 * i + 2, 0) = yawAxis.X;
		Jacobian(3 * i + 2, 1) = yawAxis.Y;
		Jacobian(3 * i + 2, 2) = yawAxis.Z;
	}
}

void ChainData::TransformSegments(const FVector& origin)
{
	for (int i = 0; i < this->NumberOfSegments; ++i)
	{
		FVector posStart = origin;
		if (i > 0) {
			posStart = this->SegmentGlobalTransforms[i - 1].TransformPosition(posStart);
		}
		FVector posEnd = this->SegmentGlobalTransforms[i].TransformPosition(origin);

		this->ChildSegments[i]->SetActorLocation(posStart);

		FVector dir = (posEnd - posStart).GetSafeNormal();
		FQuat rot = FQuat::FindBetween(FVector::UpVector, dir);
		this->ChildSegments[i]->SetActorRotation(rot);
	}
}
