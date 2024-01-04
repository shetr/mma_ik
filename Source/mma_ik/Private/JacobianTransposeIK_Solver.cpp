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

void AJacobianTransposeIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime)
{
    double h = 0.0001f;
    double EPS = 0.001f;
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);
    // TODO: delete this line later
    data.RecomputeJacobian();
    int iter = 0;

    #if 0
    UE_LOG(LogTemp, Warning, TEXT("data.EndEffectorPos: %f, %f, %f"), data.EndEffectorPos.X, data.EndEffectorPos.Y, data.EndEffectorPos.Z);
    #endif
    while ((data.EndEffectorPos - targetPosition).Length() > EPS)
    {
        if (iter > 0)
            break;
        data.RecomputeJacobian();
        data.Jacobian.Transpose(JacobianTranspose);
        dX.Set(targetPosition - data.EndEffectorPos);
        JacobianTranspose.Multiply(dX, dO);
    
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

