// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "G_Chain.generated.h"

UCLASS()
class MMA_IK_API AG_Chain : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AG_Chain();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
