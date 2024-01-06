// Fill out your copyright notice in the Description page of Project Settings.


#include "JacobianInverseIK_Solver.h"

AJacobianInverseIK_Solver::AJacobianInverseIK_Solver()
    : dX(3)
{
}

void AJacobianInverseIK_Solver::BeginPlay()
{
}

void AJacobianInverseIK_Solver::Tick(float DeltaTime)
{
}

void AJacobianInverseIK_Solver::Reset(ChainData& data)
{
}

void AJacobianInverseIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime, bool CCDTopDown)
{
    double EPS = 0.001f;
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    double originTargetDist = (targetPosition - origin).Length();
    if (originTargetDist > data.TotalChainLength) {
        FVector targetDir = targetPosition - origin;
        targetDir = targetDir * (data.TotalChainLength / originTargetDist);
        targetPosition = origin + targetDir;
    }
    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);
    dX.Set(targetPosition - data.EndEffectorPos);

    if ((data.EndEffectorPos - targetPosition).Length() > EPS)
    {
        data.RecomputeJacobian();
        data.Jacobian.Transpose(JT);
        data.Jacobian.Multiply(JT, JJT);
        double determinant = JJT.Determinant3x3();
        UE_LOG(LogTemp, Warning, TEXT("data.EndEffectorPos: %f, %f, %f"), data.EndEffectorPos.X, data.EndEffectorPos.Y, data.EndEffectorPos.Z);
        #if 0
        UE_LOG(LogTemp, Warning, TEXT("JJT 0: %f, %f, %f"), JJT(0, 0), JJT(1, 0), JJT(2, 0));
        UE_LOG(LogTemp, Warning, TEXT("JJT 1: %f, %f, %f"), JJT(0, 1), JJT(1, 1), JJT(2, 1));
        UE_LOG(LogTemp, Warning, TEXT("JJT 2: %f, %f, %f"), JJT(0, 2), JJT(1, 2), JJT(2, 2));
        #endif

        if (!JJT.OnlyZeros()) {
            if (abs(determinant) == 0.0) {
                UE_LOG(LogTemp, Warning, TEXT("zero determinant"));
                ReduceJacobian(data.Jacobian);

                RJ.Transpose(JT);
                dX.Clone(RdX);
                RJ.Multiply(JT, JJT);
            }
            JJT.Inverse(JJTinv);
            JT.Multiply(JJTinv, Jpinv);
            Jpinv.Multiply(dX, dO);

            for (int i = 0; i < data.NumberOfSegments; ++i)
            {
                data.SegmentAngles[i].X += dO[3 * i + 0] * DeltaTime;
                data.SegmentAngles[i].Y += dO[3 * i + 1] * DeltaTime;
                data.SegmentAngles[i].Z += dO[3 * i + 2] * DeltaTime;
                #if 0
                UE_LOG(LogTemp, Warning, TEXT("angles: %f, %f, %f"), data.SegmentAngles[i].X, data.SegmentAngles[i].Y, data.SegmentAngles[i].Z);
                #endif
            }
            data.RecomputeSegmentTransforms();
            data.TransformSegments(origin);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("zero jacobian"));
        }
    }
}

void AJacobianInverseIK_Solver::ReduceJacobian(const MatrixMxN& J)
{
    GJ.Clone(J);
    GdX.Clone(dX);

    // Gaussian ellimination
    int ph = 0;
    int pw = 0;
    for (; ph < GJ.GetHeight() && pw < GJ.GetWidth(); ++ph, ++pw) {
        while (pw < GJ.GetWidth() && GJ(pw, ph) == 0.0) {
            bool nonZeroFound = false;
            for (int h = ph + 1; h < GJ.GetHeight(); ++h) {
                if (GJ(pw, h) != 0.0) {
                    GJ.SwapRows(ph, h);
                    nonZeroFound = true;
                    break;
                }
            }
            if (!nonZeroFound) {
                ++pw;
            }
        }

        for (int h = ph + 1; h < GJ.GetHeight(); ++h) {
            if (GJ(pw, h) == 0.0) {
                continue;
            }
            double mul = GJ(pw, h) / GJ(pw, ph);
            for (int w = pw; w < GJ.GetWidth(); ++w) {
                GJ(w, h) -= mul * GJ(w, ph);
            }
            GdX[h] -= mul * GdX[ph];
        }
    }
    // check number of zero rows
    int zero_rows = 0;
    for (int h = 0; h < GJ.GetHeight(); ++h) {
        bool only_zeros = true;
        for (int w = 0; w < GJ.GetWidth(); ++w) {
            if (GJ(w, h) != 0.0) {
                only_zeros = false;
            }
        }
        if (only_zeros) {
            zero_rows++;
        }
    }
    int pivot_count = GJ.GetHeight() - zero_rows;
    UE_LOG(LogTemp, Warning, TEXT("pivot_count: %d"), pivot_count);
    // Remove rows containing only zeros
    RJ.SetNum(GJ.GetWidth(), pivot_count);
    RdX.Reset(pivot_count);
    RJ.ClonePart(GJ);
    RdX.ClonePart(GdX);
}
