// Fill out your copyright notice in the Description page of Project Settings.


#include "G_Chain.h"

// Sets default values
AG_Chain::AG_Chain()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

// Called when the game starts or when spawned
void AG_Chain::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AG_Chain::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

