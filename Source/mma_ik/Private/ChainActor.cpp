// Fill out your copyright notice in the Description page of Project Settings.


#include "ChainActor.h"

// Sets default values
AChainActor::AChainActor()
    : AActor()
{
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    //ChainMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CubeMesh"));
    //RootComponent = ChainMesh;

 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    ChainSegment = ConstructorHelpers::FClassFinder<AStaticMeshActor>(TEXT("/Script/Engine.Blueprint'/Game/ChainSegment.ChainSegment'")).Class;

}

// Called when the game starts or when spawned
void AChainActor::BeginPlay()
{
	Super::BeginPlay();
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    if (!ChainSegment) {
        ChainSegment = ConstructorHelpers::FClassFinder<AStaticMeshActor>(TEXT("/Script/Engine.Blueprint'/Game/ChainSegment.ChainSegment'")).Class;
    }

    data.TotalChainLength = TotalChainLength;
    data.NumberOfSegments = NumberOfSegments;

    GenerateSegments();

    if (TargetPointClass)
    {
        FVector cLoc = FVector(0.0f, 0.0f, data.TotalChainLength);
        data.TargetPoint = GetWorld()->SpawnActor<AStaticMeshActor>(TargetPointClass, GetActorLocation() + cLoc, {});
        data.TargetPoint->SetMobility(EComponentMobility::Type::Movable);
    }

    if (SolverClass)
    {
        solver = GetWorld()->SpawnActor<AIK_SolverBase>(SolverClass, GetActorLocation(), {});
    }
}

void AChainActor::GenerateSegments()
{
    float segmentLength = data.GetSegmentLength();

    for (int i = 0; i < data.NumberOfSegments; i++)
    {
        FVector cLoc = FVector(0.0f, 0.0f, segmentLength * i);

        FActorSpawnParameters spawnParameters;
        spawnParameters.Owner = this;
        AStaticMeshActor* ChildSegment = GetWorld()->SpawnActor<AStaticMeshActor>(ChainSegment, GetActorLocation() + cLoc, {}, spawnParameters);

        ChildSegment->SetMobility(EComponentMobility::Movable);

        FVector childOrigin;
        FVector childExtent;
        ChildSegment->GetActorBounds(false, childOrigin, childExtent);
        float childHeight = childExtent.Z * 2.0f;
        float zScale = segmentLength / childHeight;
        FVector scale = ChildSegment->GetActorScale3D();
        scale.Z = zScale;
        ChildSegment->SetActorScale3D(scale);

        // Add the child cube to the array
        data.ChildSegments.Add(ChildSegment);

        //ChildSegment->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
    }
    data.EndEffectorPos = FVector(0.0f, 0.0f, TotalChainLength);
    data.Reset();
}

// Called every frame
void AChainActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    //UE_LOG(LogTemp, Warning, TEXT("Test log 2"));

    if (data.NumberOfSegments != NumberOfSegments || data.TotalChainLength != TotalChainLength) {
        for (int i = 0; i < data.NumberOfSegments; i++) {
            data.ChildSegments[i]->Destroy();
        }
        data.ChildSegments.Empty();
        data.TotalChainLength = TotalChainLength;
        data.NumberOfSegments = NumberOfSegments;
        GenerateSegments();
        data.Reset();
    }

    if (solver) {
        solver->Solve(data, GetActorLocation(), DeltaTime);
    }

}