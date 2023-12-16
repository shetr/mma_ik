// Fill out your copyright notice in the Description page of Project Settings.


#include "JacobianTransposeIK_Solver.h"

AJacobianTransposeIK_Solver::AJacobianTransposeIK_Solver()
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
    //while (abs(endEffectorPosition — targetPosition) > EPS) {
    //    dO = GetDeltaOrientation();
    //    O += dO * h; // T=O+dO*h
    //}
}

//Vector GetDeltaOrientation() {
//    Jt = GetJacobianTranspose();
//    V = targetPosition — endEffectorPosition;
//    dO = Jt * V; // Matrix-Vector Mult.
//    return dO;
//}

//Matrix GetJacobianTranspose() {
//    J_A = CrossProduct(rotAxisA, endEffectorPos — jointAPos);
//    J_B = CrossProduct(rotAxisB, endEffectorPos — jointBPos);
//    J_C = CrossProduct(rotAxisC, endEffectorPos — jointCPos);    J = new Matrix();
//    J.addColumn(J_A);
//    J.addColumn(J_B);
//    J.addColumn(J_C);
//    return J.transpose();
//}