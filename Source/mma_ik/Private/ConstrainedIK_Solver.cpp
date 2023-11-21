// Fill out your copyright notice in the Description page of Project Settings.


#include "ConstrainedIK_Solver.h"

#include "MathUtil.h"

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

void AConstrainedIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime)
{
	FVector targetPos = data.TargetPoint->GetActorLocation();
    FVector targetDir = (targetPos - origin).GetSafeNormal();
    float targetDist = (targetPos - origin).Length();

    float segmentLength = data.GetSegmentLength();

    if (targetDist >= data.TotalChainLength)
    {
        for (int i = 0; i < data.NumberOfSegments; i++)
        {
            FVector loc = origin + (segmentLength * i) * targetDir;
            AStaticMeshActor* childSegment = data.ChildSegments[i];
            childSegment->SetActorLocation(loc);

            FQuat rot = FQuat::FindBetween(FVector::UpVector, targetDir);
            childSegment->SetActorRotation(rot);
        }
    }
    else
    {
        float arcAngle = AngleOptimum(targetDist / data.TotalChainLength, data.NumberOfSegments);
        float arcRadius = targetDist / (2 * sin(arcAngle * 0.5f));
        FVector rotVec = FVector::CrossProduct(FVector::UpVector, targetDir).GetSafeNormal();
        FVector upDir = -FVector::CrossProduct(targetDir, rotVec).GetSafeNormal();
        float upDist = sqrt(arcRadius * arcRadius - (targetDist * 0.5f) * (targetDist * 0.5f));
        if (arcAngle > PI) {
            upDist = -upDist;
        }
        FVector arcOrigin = origin + ((targetDist * 0.5f) * targetDir) + (upDist * upDir);
        float startAngle = (2.0 * PI - arcAngle) * 0.5f;
        float angleStep = arcAngle / data.NumberOfSegments;
        for (int i = 0; i < data.NumberOfSegments; i++)
        {
            float angle0 = startAngle + i * angleStep;
            float angle1 = startAngle + (i + 1) * angleStep;
            FVector loc0 = arcOrigin + arcRadius * (cos(angle0) * upDir + sin(angle0) * targetDir);
            FVector loc1 = arcOrigin + arcRadius * (cos(angle1) * upDir + sin(angle1) * targetDir);
            AStaticMeshActor* childSegment = data.ChildSegments[i];
            childSegment->SetActorLocation(loc0);

            FVector dir = (loc1 - loc0).GetSafeNormal();
            FQuat rot = FQuat::FindBetween(FVector::UpVector, dir);
            childSegment->SetActorRotation(rot);
        }
    }

}

float AConstrainedIK_Solver::AngleObjective(float t, int n, float alphaHalf) const
{

    return sin(alphaHalf) - t * n * sin(alphaHalf / n);
}

float AConstrainedIK_Solver::AngleOptimum(float t, int n) const
{
    float min = 0.000001f;
    float max = PI;
    float fmin = AngleObjective(t, n, min);
    float fmax = AngleObjective(t, n, max);
    float epsilon = 0.001f;
    while (max - min > epsilon) {
        float mid = (min + max) * 0.5f;
        float fmid = AngleObjective(t, n, mid);
        if (fmin * fmid <= 0) {
            max = mid;
            fmax = fmid;
        } else {
            min = mid;
            fmin = fmid;
        }
    }
    float alphaHalf = (min + max) * 0.5f;
    return alphaHalf * 2.0f;
}
