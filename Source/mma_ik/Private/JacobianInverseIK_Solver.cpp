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
    double EPS = 0.001f;
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);
    dX.Set(targetPosition - data.EndEffectorPos);

    if ((data.EndEffectorPos - targetPosition).Length() > EPS)
    {
        data.RecomputeJacobian();
        data.Jacobian.Transpose(JacobianTranspose);
        data.Jacobian.Multiply(JacobianTranspose, JJT);
        double determinant = JJT.Determinant3x3();
        UE_LOG(LogTemp, Warning, TEXT("data.EndEffectorPos: %f, %f, %f"), data.EndEffectorPos.X, data.EndEffectorPos.Y, data.EndEffectorPos.Z);
        #if 1
        UE_LOG(LogTemp, Warning, TEXT("JJT 0: %f, %f, %f"), JJT(0, 0), JJT(1, 0), JJT(2, 0));
        UE_LOG(LogTemp, Warning, TEXT("JJT 1: %f, %f, %f"), JJT(0, 1), JJT(1, 1), JJT(2, 1));
        UE_LOG(LogTemp, Warning, TEXT("JJT 2: %f, %f, %f"), JJT(0, 2), JJT(1, 2), JJT(2, 2));
        #endif

        if (!JJT.OnlyZeros()) {
            if (abs(determinant) == 0.0) {
                UE_LOG(LogTemp, Warning, TEXT("zero determinant"));
                ReduceJacobian(data.Jacobian);
                for (int i = 0; i < data.NumberOfSegments; ++i)
                {
                    UE_LOG(LogTemp, Warning, TEXT("RJ: %f, %f, %f"), RJ(i, 0), RJ(i, 1), RJ(i, 2));
                }

                CJ.Transpose(JacobianTranspose);
                dX.Clone(CdX);
                CJ.Multiply(JacobianTranspose, JJT);
            }
            JJT.Inverse(JJTinv);
            JacobianTranspose.Multiply(JJTinv, Jpinv);
            Jpinv.Multiply(dX, dO);

            for (int i = 0; i < data.NumberOfSegments; ++i)
            {
                data.SegmentAngles[i].X += dO[3 * i + 0] * DeltaTime;
                data.SegmentAngles[i].Y += dO[3 * i + 1] * DeltaTime;
                data.SegmentAngles[i].Z += dO[3 * i + 2] * DeltaTime;
                UE_LOG(LogTemp, Warning, TEXT("angles: %f, %f, %f"), data.SegmentAngles[i].X, data.SegmentAngles[i].Y, data.SegmentAngles[i].Z);
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
    RJ.Clone(J);
    RdX.Clone(dX);

    // Gaussian ellimination
    int ph = 0;
    int pw = 0;
    for (; ph < RJ.GetHeight() && pw < RJ.GetWidth(); ++ph, ++pw) {
        while (pw < RJ.GetWidth() && RJ(pw, ph) == 0.0) {
            bool nonZeroFound = false;
            for (int h = ph + 1; h < RJ.GetHeight(); ++h) {
                if (RJ(pw, h) != 0.0) {
                    RJ.SwapRows(ph, h);
                    nonZeroFound = true;
                    break;
                }
            }
            if (!nonZeroFound) {
                ++pw;
            }
        }

        for (int h = ph + 1; h < RJ.GetHeight(); ++h) {
            if (RJ(pw, h) == 0.0) {
                continue;
            }
            double mul = RJ(pw, h) / RJ(pw, ph);
            for (int w = pw; w < RJ.GetWidth(); ++w) {
                RJ(w, h) -= mul * RJ(w, ph);
            }
            RdX[h] -= mul * RdX[ph];
        }
    }
    // check number of zero rows
    int zero_rows = 0;
    for (int h = 0; h < RJ.GetHeight(); ++h) {
        bool only_zeros = true;
        for (int w = 0; w < RJ.GetWidth(); ++w) {
            if (RJ(w, h) != 0.0) {
                only_zeros = false;
            }
        }
        if (only_zeros) {
            zero_rows++;
        }
    }
    int pivot_count = RJ.GetHeight() - zero_rows;
    UE_LOG(LogTemp, Warning, TEXT("pivot_count: %d"), pivot_count);

    CJ.SetNum(RJ.GetWidth(), pivot_count);
    CdX.Reset(pivot_count);
    CJ.ClonePart(RJ);
    CdX.ClonePart(RdX);
}

void AJacobianInverseIK_Solver::GetPseudoinv(const MatrixMxN& J, const MatrixMxN& JT)
{
    check(JJT.GetWidth() == 3);
    check(JJT.GetHeight() == 3);

    double a = -1.;
    double b = JJT(0, 0) + JJT(1, 1) + JJT(2, 2);
    double c = (-JJT(0, 0) * JJT(1, 1) - JJT(0, 0) * JJT(2, 2) - JJT(1, 1) * JJT(2, 2) + JJT(0, 1) * JJT(0, 1) + JJT(0, 2) * JJT(0, 2) + JJT(1, 2) * JJT(1, 2));

    double D = b * b - 4.0 * a * c;
    if (D < 0) {
        UE_LOG(LogTemp, Warning, TEXT("negative discriminant"));
        return;
    }
    double sqrtD = sqrt(D);
    double l1 = (-b + sqrtD) / (2. * a);
    double l2 = (-b - sqrtD) / (2. * a);
    if (l1 < l2) {
        Swap(l1, l2);
    }
    double l[2] = { l1, l2 };
    UE_LOG(LogTemp, Warning, TEXT("l1: %f"), l1);
    UE_LOG(LogTemp, Warning, TEXT("l2: %f"), l2);

    double s1 = sqrt(l1);
    double s2 = sqrt(l2);
    double s[2] = { s1, s2 };

    JS.SetNum(2, 2);
    JS(0, 0) = s1;
    JS(1, 1) = s2;

    JT.Multiply(J, JTJ);
    JV.SetNum(2, J.GetWidth());

    for (int vi = 0; vi < 2; ++vi) {
        Temp.Clone(JTJ);
        for (int i = 0; i < Temp.GetWidth(); ++i)
        {
            Temp(i, i) = Temp(i, i) - l[vi];
        }

        // Gaussian ellimination
        int ph = 0;
        int pw = 0;
        for (; ph < Temp.GetHeight() && pw < Temp.GetWidth(); ++ph, ++pw) {
            while (Temp(pw, ph) == 0.0 && pw < Temp.GetWidth()) {
                bool nonZeroFound = false;
                for (int h = ph + 1; h < Temp.GetHeight(); ++h) {
                    if (Temp(pw, h) != 0.0) {
                        Temp.SwapRows(ph, h);
                        nonZeroFound = true;
                        break;
                    }
                }
                if (!nonZeroFound) {
                    ++pw;
                }
            }

            for (int h = ph + 1; h < Temp.GetHeight(); ++h) {
                if (Temp(pw, h) == 0.0) {
                    continue;
                }
                double mul = Temp(pw, h) / Temp(pw, ph);
                for (int w = pw; w < Temp.GetWidth(); ++w) {
                    Temp(w, h) -= mul * Temp(w, ph);
                }
            }
        }
        int pivot_count = ph;
        UE_LOG(LogTemp, Warning, TEXT("pivot_count: %d"), pivot_count);
    }



    JV.Transpose(JVT);
    J.Multiply(JV, JU);
    for (int h = 0; h < JU.GetHeight(); ++h) {
        for (int w = 0; w < JU.GetWidth(); ++w) {
            JU(w, h) /= s[w];
        }
    }
    // Jpinv = JU * JS * JV^T
    JS.Multiply(JVT, Temp);
    JU.Multiply(Temp, Jpinv);
}
