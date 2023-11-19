// Fill out your copyright notice in the Description page of Project Settings.


#include "ChainActor.h"

// Sets default values
AChainActor::AChainActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

    static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/Script/Engine.StaticMesh'/Engine/EngineMeshes/Cube.Cube'"));

    if (MeshAsset.Succeeded())
    {
        ChainMesh->SetStaticMesh(MeshAsset.Object);
    }
}

// Called when the game starts or when spawned
void AChainActor::BeginPlay()
{
	Super::BeginPlay();
	
    int numSegments = 10;
    for (int i = 0; i < numSegments; i++)
    {
        FVector cLoc = FVector(0.0f, 0.0f, 10.f * i);
        CreateChildCube(cLoc);
    }
}

// Called every frame
void AChainActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AChainActor::CreateChildCube(const FVector& RelativeLocation)
{
    // Create a child cube
    AChainActor* ChildCube = GetWorld()->SpawnActor<AChainActor>(FVector::ZeroVector, FRotator::ZeroRotator);

    // Add the child cube to the array
    childChains.Add(ChildCube);

    // Attach the child cube to the root cube
    ChildCube->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);

    // Set the relative location of the child cube
    ChildCube->SetActorRelativeLocation(RelativeLocation);
}