// Fill out your copyright notice in the Description page of Project Settings.


#include "JacobianTransposeIK_Solver.h"

AJacobianTransposeIK_Solver::AJacobianTransposeIK_Solver()
    : dX(3)
{
}

void AJacobianTransposeIK_Solver::BeginPlay()
{
}

void AJacobianTransposeIK_Solver::Tick(float DeltaTime)
{
}

void AJacobianTransposeIK_Solver::Reset(ChainData& data)
{
}

void AJacobianTransposeIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime, bool CCDTopDown)
{
    double h = 0.0001;
    double EPS = 0.001;
    int maxIters = 5;
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    // If target is too far, move it closer
    double originTargetDist = (targetPosition - origin).Length();
    if (originTargetDist > data.TotalChainLength) {
        FVector targetDir = targetPosition - origin;
        targetDir = targetDir * (data.TotalChainLength / originTargetDist);
        targetPosition = origin + targetDir;
    }
    // recompute transforms
    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);

    #if 0
    UE_LOG(LogTemp, Warning, TEXT("data.EndEffectorPos: %f, %f, %f"), data.EndEffectorPos.X, data.EndEffectorPos.Y, data.EndEffectorPos.Z);
    #endif
    int iter = 0;
    while ((data.EndEffectorPos - targetPosition).Length() > EPS && iter < maxIters)
    {
        // compute jacobian
        data.RecomputeJacobian();
        // apply the jacobian transpose: dO = J^T * dX
        data.Jacobian.Transpose(JacobianTranspose);
        dX.Set(targetPosition - data.EndEffectorPos);
        JacobianTranspose.Multiply(dX, dO);

        // apply the angle changes
        for (int i = 0; i < data.NumberOfSegments; ++i)
        {
            data.SegmentAngles[i].X += dO[3 * i + 0] * h * DeltaTime;
            data.SegmentAngles[i].Y += dO[3 * i + 1] * h * DeltaTime;
            data.SegmentAngles[i].Z += dO[3 * i + 2] * h * DeltaTime;
            #if 0
            UE_LOG(LogTemp, Warning, TEXT("angles: %f, %f, %f"), data.SegmentAngles[i].Pitch, data.SegmentAngles[i].Roll, data.SegmentAngles[i].Yaw);
            #endif
        }
        data.RecomputeSegmentTransforms();
        data.TransformSegments(origin);
        iter++;
    }
}

