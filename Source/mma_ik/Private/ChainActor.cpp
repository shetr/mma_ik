// Fill out your copyright notice in the Description page of Project Settings.


#include "ChainActor.h"

// Sets default values
AChainActor::AChainActor()
{
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    //ChainMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CubeMesh"));
    //RootComponent = ChainMesh;

 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

    ChainSegment = ConstructorHelpers::FClassFinder<AStaticMeshActor>(TEXT("/Script/Engine.Blueprint'/Game/ChainSegment.ChainSegment'")).Class;

}

// Called when the game starts or when spawned
void AChainActor::BeginPlay()
{
	Super::BeginPlay();

    float segmentLength = TotalChainLength / NumberOfSegments;
	
    for (int i = 0; i < NumberOfSegments; i++)
    {
        FVector cLoc = FVector(0.0f, 0.0f, segmentLength * i);

        FActorSpawnParameters spawnParameters;
        spawnParameters.Owner = this;
        AStaticMeshActor* ChildSegment = GetWorld()->SpawnActor<AStaticMeshActor>(ChainSegment, GetActorLocation() + cLoc, {}, spawnParameters);

        FVector childOrigin;
        FVector childExtent;
        ChildSegment->GetActorBounds(false, childOrigin, childExtent);
        float childHeight = childExtent.Z * 2.0f;
        float zScale = segmentLength / childHeight;
        FVector scale = ChildSegment->GetActorScale3D();
        scale.Z = zScale;
        ChildSegment->SetActorScale3D(scale);

        // Add the child cube to the array
        ChildSegments.Add(ChildSegment);

        //ChildSegment->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
    }
}

// Called every frame
void AChainActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}