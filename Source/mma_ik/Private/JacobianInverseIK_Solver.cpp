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
    FVector targetPosition = data.TargetPoint->GetActorLocation();
    dX.Set(targetPosition - data.EndEffectorPos);

    data.RecomputeSegmentTransforms();
    data.TransformSegments(origin);
    data.RecomputeJacobian();
    data.Jacobian.Transpose(JacobianTranspose);
    data.Jacobian.Multiply(JacobianTranspose, JJT);
    double determinant = JJT.Determinant3x3();
    UE_LOG(LogTemp, Warning, TEXT("data.EndEffectorPos: %f, %f, %f"), data.EndEffectorPos.X, data.EndEffectorPos.Y, data.EndEffectorPos.Z);
#if 0
    UE_LOG(LogTemp, Warning, TEXT("JJT 0: %f, %f, %f"), JJT(0, 0), JJT(1, 0), JJT(2, 0));
    UE_LOG(LogTemp, Warning, TEXT("JJT 1: %f, %f, %f"), JJT(0, 1), JJT(1, 1), JJT(2, 1));
    UE_LOG(LogTemp, Warning, TEXT("JJT 2: %f, %f, %f"), JJT(0, 2), JJT(1, 2), JJT(2, 2));
#endif
    if (abs(determinant) > 0.0) {
        JJT.Inverse3x3(JJTinv);
        JacobianTranspose.Multiply(JJTinv, Jpinv);
        Jpinv.Multiply(dX, dO);

        for (int i = 0; i < data.NumberOfSegments; ++i)
        {
            data.SegmentAngles[i].Pitch += dO[3 * i + 0] * DeltaTime;
            data.SegmentAngles[i].Roll += dO[3 * i + 1] * DeltaTime;
            data.SegmentAngles[i].Yaw += dO[3 * i + 2] * DeltaTime;
            UE_LOG(LogTemp, Warning, TEXT("angles: %f, %f, %f"), data.SegmentAngles[i].Pitch, data.SegmentAngles[i].Roll, data.SegmentAngles[i].Yaw);
        }
        data.RecomputeSegmentTransforms();
        data.TransformSegments(origin);
    }
    else if (!JJT.OnlyZeros()) {
        UE_LOG(LogTemp, Warning, TEXT("zero determinant"));
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("zero jacobian"));
    }

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

    double s1 = sqrt(l1);
    double s2 = sqrt(l2);

    JS.SetNum(2, 2);
    JS(0, 0) = s1;
    JS(1, 1) = s2;

    JT.Multiply(J, JTJ);
    JV.SetNum(2, J.GetWidth());
}
