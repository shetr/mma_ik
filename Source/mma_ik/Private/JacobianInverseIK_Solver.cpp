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

void AJacobianInverseIK_Solver::Solve(ChainData& data, const FVector& origin, float DeltaTime)
{
    double EPS = 0.1;
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
    dX.Set(targetPosition - data.EndEffectorPos);

    int iter = 0;
    while ((data.EndEffectorPos - targetPosition).Length() > EPS && iter < maxIters)
    {
        // We want to solve equation J * dO = dX
        // We do it by using pseudoinverse, which is derived from minimal norm solution formuala dO = J^T * (J * J^T)^-1 * dX
        data.RecomputeJacobian();
        data.Jacobian.Transpose(JT);
        data.Jacobian.Multiply(JT, JJT);
        double determinant = JJT.Determinant3x3();
        #if 0
        UE_LOG(LogTemp, Warning, TEXT("data.EndEffectorPos: %f, %f, %f"), data.EndEffectorPos.X, data.EndEffectorPos.Y, data.EndEffectorPos.Z);
        #endif
        
        if (!JJT.OnlyZeros()) {
            // If there are some singularities in (J * J^T), then reduce the jacobian, so that we can do the pseudoinverse
            if (abs(determinant) == 0.0) {
                UE_LOG(LogTemp, Warning, TEXT("zero determinant"));
                ReduceJacobian(data.Jacobian);

                #if 0
                for (int w = 0; w < RJ.GetWidth(); ++w) {
                    if (RJ.GetHeight() == 2) {
                        UE_LOG(LogTemp, Warning, TEXT("RJ %d: %f %f"), w, RJ(w, 0), RJ(w, 1));
                    }
                    else {
                        UE_LOG(LogTemp, Warning, TEXT("RJ %d: %f"), w, RJ(w, 0));
                    }
                }
                if (RdX.GetSize() == 2) {
                    UE_LOG(LogTemp, Warning, TEXT("RdX: %f %f"), RdX[0], RdX[1]);
                }
                else {
                    UE_LOG(LogTemp, Warning, TEXT("RdX: %f"), RdX[0]);
                }
                #endif

                // Recompute J matricies from RJ matricies
                RJ.Transpose(JT);
                dX.Clone(RdX);
                RJ.Multiply(JT, JJT);
            }
            // Compute and apply jacobian pseudoinverse
            JJT.Inverse(JJTinv);
            JT.Multiply(JJTinv, Jpinv);
            Jpinv.Multiply(dX, dO);

            #if 0
            if (JJT.GetHeight() == 2) {
                UE_LOG(LogTemp, Warning, TEXT("JJT 0: %f, %f"), JJT(0, 0), JJT(1, 0));
                UE_LOG(LogTemp, Warning, TEXT("JJT 1: %f, %f"), JJT(0, 1), JJT(1, 1));

                UE_LOG(LogTemp, Warning, TEXT("JJTinv 0: %f, %f"), JJTinv(0, 0), JJTinv(1, 0));
                UE_LOG(LogTemp, Warning, TEXT("JJTinv 1: %f, %f"), JJTinv(0, 1), JJTinv(1, 1));

                for (int h = 0; h < Jpinv.GetHeight(); ++h) {
                    UE_LOG(LogTemp, Warning, TEXT("Jpinv %d: %f %f"), h, Jpinv(0, h), Jpinv(1, h));
                }
            }
            else if (JJT.GetHeight() == 3) {
                UE_LOG(LogTemp, Warning, TEXT("JJT 0: %f, %f, %f"), JJT(0, 0), JJT(1, 0), JJT(2, 0));
                UE_LOG(LogTemp, Warning, TEXT("JJT 1: %f, %f, %f"), JJT(0, 1), JJT(1, 1), JJT(2, 1));
                UE_LOG(LogTemp, Warning, TEXT("JJT 2: %f, %f, %f"), JJT(0, 2), JJT(1, 2), JJT(2, 2));

                for (int h = 0; h < Jpinv.GetHeight(); ++h) {
                    UE_LOG(LogTemp, Warning, TEXT("Jpinv %d: %f %f %f"), h, Jpinv(0, h), Jpinv(1, h), Jpinv(2, h));
                }
            }
            #endif

            // dampen the angle change if it's too high (prevents insane twitching movements)
            double maxAllowedAngleChange = 2.0;
            double maxAngle = 0.0;
            for (int i = 0; i < dO.GetSize(); ++i) {
                if (abs(dO[i]) > maxAngle) {
                    maxAngle = abs(dO[i]);
                }
            }
            if (maxAngle > maxAllowedAngleChange) {
                double scale = maxAllowedAngleChange / maxAngle;
                for (int i = 0; i < dO.GetSize(); ++i) {
                    dO[i] *= scale;
                }
            }

            // apply the angle changes
            for (int i = 0; i < data.NumberOfSegments; ++i)
            {
                data.SegmentAngles[i].X += dO[3 * i + 0] * DeltaTime;
                data.SegmentAngles[i].Y += dO[3 * i + 1] * DeltaTime;
                data.SegmentAngles[i].Z += dO[3 * i + 2] * DeltaTime;
                #if 0
                UE_LOG(LogTemp, Warning, TEXT("dO: %f, %f, %f"), dO[3 * i + 0], dO[3 * i + 1], dO[3 * i + 2]);
                #endif
                #if 0
                UE_LOG(LogTemp, Warning, TEXT("angles: %f, %f, %f"), data.SegmentAngles[i].X, data.SegmentAngles[i].Y, data.SegmentAngles[i].Z);
                #endif
            }
            data.RecomputeSegmentTransforms();
            data.TransformSegments(origin);
        }
        else {
            // hopefully doesn't happen, but we can't do anything about it if it does happen
            UE_LOG(LogTemp, Warning, TEXT("zero jacobian"));
            break;
        }
        ++iter;
    }
    //UE_LOG(LogTemp, Warning, TEXT("iters: %d"), iter);
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
                    GdX.SwapElems(ph, h);
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
