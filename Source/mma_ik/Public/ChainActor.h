// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "G_Chain.h"
#include "ChainData.h"
#include "IK_SolverBase.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/StaticMeshActor.h"
#include "ChainActor.generated.h"

UCLASS()
class MMA_IK_API AChainActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AChainActor();


	UPROPERTY(EditAnywhere, Category = "Segments")
	TSubclassOf<AStaticMeshActor> ChainSegment;

	UPROPERTY(EditAnywhere, Category = "Segments")
	TSubclassOf<AStaticMeshActor> TargetPointClass;

	UPROPERTY(EditAnywhere, Category = "Segments")
	TSubclassOf<AIK_SolverBase> SolverClass;

	UPROPERTY(EditAnywhere, Category = "Segments")
	float TotalChainLength = 200.0f;

	UPROPERTY(EditAnywhere, Category = "Segments")
	int32 NumberOfSegments = 10;

private:

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	void GenerateSegments();

	ChainData data;
	AIK_SolverBase* solver = nullptr;
public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
