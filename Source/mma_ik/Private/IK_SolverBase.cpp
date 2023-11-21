// Fill out your copyright notice in the Description page of Project Settings.


#include "IK_SolverBase.h"

// Sets default values
AIK_SolverBase::AIK_SolverBase()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AIK_SolverBase::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AIK_SolverBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AIK_SolverBase::Reset(ChainData& data)
{
}

void AIK_SolverBase::Solve(ChainData& data, const FVector& origin, float DeltaTime)
{
}

