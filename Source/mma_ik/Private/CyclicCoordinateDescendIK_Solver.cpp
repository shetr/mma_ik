// Fill out your copyright notice in the Description page of Project Settings.


#include "CyclicCoordinateDescendIK_Solver.h"

ACyclicCoordinateDescendIK_Solver::ACyclicCoordinateDescendIK_Solver()
{
}

void ACyclicCoordinateDescendIK_Solver::BeginPlay()
{
}

void ACyclicCoordinateDescendIK_Solver::Tick(float DeltaTime)
{
}

void ACyclicCoordinateDescendIK_Solver::Reset(ChainData& data)
{
}

void ACyclicCoordinateDescendIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime, bool CCDTopDown)
{
    float EPS = 0.001f;
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);
    
    int iter = 0;
    int maxIters = 60;

    bool shouldStop = false;

    while (iter < maxIters)
    {
        if (shouldStop)
        {
            shouldStop = false;
            break;
        }

        int i;
        if (CCDTopDown)
            i = data.NumberOfSegments - 1;
        else
            i = 0;

        while (true)
        {
            if (CCDTopDown)
            {
                if (i < 0)
                    break;
            }
            else
            {
                if (i >= data.NumberOfSegments)
                    break;
            }

            FVector currPosition = data.ChildSegments[i]->GetActorLocation();
            FVector currToEnd = data.EndEffectorPos - currPosition;
            FVector currToTarget = targetPosition - currPosition;

            // Iteration angle computation
            double cosangle = FVector::DotProduct(currToEnd / currToEnd.Length(), currToTarget / currToTarget.Length());
            FVector axis = FVector::CrossProduct(currToEnd / currToEnd.Length(), currToTarget / currToTarget.Length());
            axis.Normalize();

            double angle = acos(cosangle);
            double angleRad = FMath::DegreesToRadians(angle);
            FQuat rotationQuat = FQuat(axis, angleRad);

            // Location computation with quaternions
            for (size_t j = i + 1; j < data.NumberOfSegments; j++)
            {
                FVector tGlob = data.ChildSegments[j]->GetActorLocation();

                FVector cLoc = tGlob - currPosition;
                FVector r = rotationQuat.RotateVector(cLoc);

                data.ChildSegments[j]->SetActorLocation(currPosition + r);
            }

            // Rotation computation
            for (size_t j = i; j < data.NumberOfSegments - 1; j++)
            {
                FVector tGlob = data.ChildSegments[j + 1]->GetActorLocation();
                FVector prevGlob = data.ChildSegments[j]->GetActorLocation();

                FVector dir = (tGlob - prevGlob).GetSafeNormal();
                FQuat rot = FQuat::FindBetween(FVector::UpVector, dir);
                data.ChildSegments[j]->SetActorRotation(rot);
            }

            data.EndEffectorPos = currPosition + rotationQuat.RotateVector(currToEnd);
            FVector dir = (data.EndEffectorPos - data.ChildSegments.Last()->GetActorLocation()).GetSafeNormal();
            FQuat rot = FQuat::FindBetween(FVector::UpVector, dir);
            data.ChildSegments.Last()->SetActorRotation(rot);

            if (abs(FVector(data.EndEffectorPos - targetPosition).Length()) < EPS)
            {
                shouldStop = true;
                break;
            }

            if (CCDTopDown)
                i--;
            else
                i++;
        }
        iter++;
    }
}

