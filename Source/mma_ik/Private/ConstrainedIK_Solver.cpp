// Fill out your copyright notice in the Description page of Project Settings.


#include "ConstrainedIK_Solver.h"

AConstrainedIK_Solver::AConstrainedIK_Solver()
{
}

void AConstrainedIK_Solver::BeginPlay()
{
}

void AConstrainedIK_Solver::Tick(float DeltaTime)
{
}

void AConstrainedIK_Solver::Reset(ChainData& data)
{
}

void AConstrainedIK_Solver::Solve(ChainData& data)
{
	FVector origin = data.ChildSegments[0]->GetActorLocation();
	FVector targetPos = data.TargetPoint->GetActorLocation();
    FVector targetDir = (targetPos - origin).GetSafeNormal();

    float segmentLength = data.GetSegmentLength();

    for (int i = 0; i < data.NumberOfSegments; i++)
    {
        FVector loc = origin + (segmentLength * i) * targetDir;
        AStaticMeshActor* childSegment = data.ChildSegments[i];
        childSegment->SetActorLocation(loc);

        FQuat rot = FQuat::FindBetween(FVector::UpVector, targetDir);
        childSegment->SetActorRotation(rot);
    }
}
