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

void ACyclicCoordinateDescendIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime)
{
    float EPS = 0.001f;
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);
    
    int iter = 0;
    int maxIters = 10;

    bool isBackward = true;

    bool shouldStop = false;
    if (isBackward)
    {
        while (iter < maxIters)
        {
            if (shouldStop)
            {
                shouldStop = false;
                break;
            }
            for (int i = data.NumberOfSegments - 1; i > 0; --i)
            {
                FVector currToEnd = data.EndEffectorPos - data.ChildSegments[i]->GetActorLocation();
                FVector currToTarget = targetPosition - data.ChildSegments[i]->GetActorLocation();
                
                double cosangle = FVector::DotProduct(currToEnd / currToEnd.Length(), currToTarget / currToTarget.Length());
                FVector axis = FVector::CrossProduct(currToEnd / currToEnd.Length(), currToTarget / currToTarget.Length());
                axis.Normalize();

                double angle = acos(cosangle);

                float angleRad = FMath::DegreesToRadians(angle);

                FQuat rotationQuat = FQuat(axis, angleRad);

                FVector rotatedCurr = FQuatRotationMatrix(rotationQuat).TransformVector(currToEnd);

                FRotator currRot = currToEnd.Rotation();
                FRotator resRot = rotatedCurr.Rotation();

                float pitchDiff = FMath::FindDeltaAngleDegrees(currRot.Pitch, resRot.Pitch);
                float rollDiff = FMath::FindDeltaAngleDegrees(currRot.Roll, resRot.Roll);
                float yawDiff = FMath::FindDeltaAngleDegrees(currRot.Yaw, resRot.Yaw);

                // TODO: fix
                data.SegmentAngles[i].Y += pitchDiff;
                data.SegmentAngles[i].Z += yawDiff;
                //data.SegmentAngles[i].Yaw += yawDiff;

                //UE_LOG(LogTemp, Warning, TEXT("Pitch: %f, Roll: %f, Yaw: %f"), data.SegmentAngles[i].Pitch, data.SegmentAngles[i].Roll, data.SegmentAngles[i].Yaw);

                if (abs(FVector(data.EndEffectorPos - targetPosition).Length()) < EPS)
                {
                    shouldStop = true;
                    break;
                }
                data.RecomputeSegmentTransforms();
                data.TransformSegments(origin);
            }

            iter++;
        }
    }
}

