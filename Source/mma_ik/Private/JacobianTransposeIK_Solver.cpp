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
    float h = 0.001f;
    float EPS = 0.001f;
    FVector lastSegmentPosition = data.ChildSegments.Last()->GetActorLocation();
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    data.RecomputeSegmentTransforms();
    int iter = 0;
    while (abs((lastSegmentPosition - targetPosition).Length() - data.GetSegmentLength()) > EPS) 
    {
        iter++;
        if (iter > 15)
            break;
        data.RecomputeJacobian(targetPosition);
        data.Jacobian.Transpose(JacobianTranspose);
        dX.Set(targetPosition - data.EndEffectorPos);
        JacobianTranspose.Multiply(dX, dO);

        for (int i = 0; i < data.NumberOfSegments; ++i)
        {
            data.SegmentAngles[i].Pitch += dO[3 * i + 0] * h;
            data.SegmentAngles[i].Roll += dO[3 * i + 1] * h;
            data.SegmentAngles[i].Yaw += dO[3 * i + 2] * h;
        }
        data.RecomputeSegmentTransforms();
        data.TransformSegments(origin);
        lastSegmentPosition = data.ChildSegments.Last()->GetActorLocation();
    }
}
