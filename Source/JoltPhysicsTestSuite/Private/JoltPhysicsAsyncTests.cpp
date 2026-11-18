// Copyright (C) 2024 Van de Walle Bastien
// SPDX-License-Identifier: MIT

// Experiments about running JPH::PhysicsSystem::Update asynchronously.
//
// Key fact these tests document: PhysicsSystem::Update() is blocking. It builds the step's job graph and then
// calls JobSystem::WaitForJobs() on a barrier before returning (PhysicsSystem.cpp), whatever the job system is.
// Multi-threading only means the *jobs inside* the step run in parallel; the caller still waits for all of them.
// So "start the physics now, end it later" (UJPRPhysicsSubsystem::StartPhysicsSimulation / ForceEndPhysicsSimulation)
// is only asynchronous if PhysicsSystem::Update itself is moved onto another thread.
//
// Run with:  Automation RunTests JoltPhysics.Async

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "JoltPhysicsTestWorld.h"

namespace JoltPhysicsTests
{
	constexpr float FixedDeltaTime = 1.f / 60.f;

	struct FCombo
	{
		EJobSystemKind JobSystem;
		EAsyncLaunch Launch;
	};

	const FCombo AllCombos[] =
	{
		{ EJobSystemKind::ThreadPool, EAsyncLaunch::DedicatedThread },
		{ EJobSystemKind::ThreadPool, EAsyncLaunch::TaskGraph },
		{ EJobSystemKind::UETask, EAsyncLaunch::DedicatedThread },
		{ EJobSystemKind::UETask, EAsyncLaunch::TaskGraph },
	};

	FString ComboLabel(const FCombo& Combo)
	{
		return FString::Printf(TEXT("[%s / %s]"), ToString(Combo.JobSystem), ToString(Combo.Launch));
	}
}

using namespace JoltPhysicsTests;

//----------------------------------------------------------------------//
// 1. Baseline: Update() blocks the caller until the whole step is done.
//----------------------------------------------------------------------//
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJoltAsyncSynchronousUpdateBlocksTest, "JoltPhysics.Async.SynchronousUpdateBlocks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FJoltAsyncSynchronousUpdateBlocksTest::RunTest(const FString& Parameters)
{
	for (const EJobSystemKind Kind : { EJobSystemKind::ThreadPool, EJobSystemKind::UETask })
	{
		const FString Label = FString::Printf(TEXT("[%s]"), ToString(Kind));

		FTestPhysicsWorld World(Kind);
		FGatedContactListener Listener;
		Listener.bBlockOnGate = false;
		World.PhysicsSystem->SetContactListener(&Listener);
		World.BuildSingleContactScene();

		const JPH::BodyID Sphere = World.SphereIds[0];
		const float StartY = static_cast<float>(World.Bodies().GetCenterOfMassPosition(Sphere).GetY());

		const double T0 = FPlatformTime::Seconds();
		const JPH::EPhysicsUpdateError Error = World.StepSync(FixedDeltaTime);
		const double ElapsedMs = (FPlatformTime::Seconds() - T0) * 1000.0;

		TestTrue(*FString::Printf(TEXT("%s Update reported no error"), *Label), Error == JPH::EPhysicsUpdateError::None);
		// Everything is already done when Update() returns: contacts were reported, bodies were integrated.
		TestTrue(*FString::Printf(TEXT("%s contacts were already reported when Update() returned"), *Label), Listener.bContactSeen.load());
		const float EndY = static_cast<float>(World.Bodies().GetCenterOfMassPosition(Sphere).GetY());
		TestTrue(*FString::Printf(TEXT("%s the sphere was already moved when Update() returned"), *Label), !FMath::IsNearlyEqual(StartY, EndY, 1e-6f));

		AddInfo(FString::Printf(TEXT("%s Update() took %.3f ms; contact callback ran on thread %u, caller thread is %u"),
			*Label, ElapsedMs, Listener.ContactThreadId.load(), FPlatformTLS::GetCurrentThreadId()));
	}
	return true;
}

//----------------------------------------------------------------------//
// 2. Moving Update() to another thread makes "start" return immediately.
//    The contact listener parks the step mid-flight, so no timing is involved:
//    if StartAsync() were blocking, the test would time out instead of passing.
//----------------------------------------------------------------------//
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJoltAsyncStartReturnsBeforeStepCompletesTest, "JoltPhysics.Async.StartReturnsBeforeStepCompletes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FJoltAsyncStartReturnsBeforeStepCompletesTest::RunTest(const FString& Parameters)
{
	for (const FCombo& Combo : AllCombos)
	{
		const FString Label = ComboLabel(Combo);

		FTestPhysicsWorld World(Combo.JobSystem);
		FGatedContactListener Listener;
		World.PhysicsSystem->SetContactListener(&Listener);
		World.BuildSingleContactScene();

		World.StartAsync(FixedDeltaTime, 1, Combo.Launch);

		// Wait until the step is provably in flight (a Jolt job is parked inside OnContactAdded).
		const double Deadline = FPlatformTime::Seconds() + 10.0;
		while (!Listener.bContactSeen.load() && FPlatformTime::Seconds() < Deadline)
		{
			FPlatformProcess::SleepNoStats(0.001f);
		}

		TestTrue(*FString::Printf(TEXT("%s step reached the contact callback while the game thread was free"), *Label), Listener.bContactSeen.load());
		TestFalse(*FString::Printf(TEXT("%s step has not finished while a job is parked"), *Label), World.IsStepFinished());
		TestTrue(*FString::Printf(TEXT("%s async step is still running"), *Label), World.IsAsyncStepRunning());
		TestTrue(*FString::Printf(TEXT("%s the contact callback ran off the game thread"), *Label),
			Listener.ContactThreadId.load() != FPlatformTLS::GetCurrentThreadId());

		Listener.OpenGate();
		World.EndAsync();

		TestTrue(*FString::Printf(TEXT("%s step is finished after EndAsync"), *Label), World.IsStepFinished());
		TestFalse(*FString::Printf(TEXT("%s the gate never timed out"), *Label), Listener.bGateTimedOut.load());
		TestTrue(*FString::Printf(TEXT("%s Update reported no error"), *Label), World.GetLastError() == JPH::EPhysicsUpdateError::None);
	}
	return true;
}

//----------------------------------------------------------------------//
// 3. Start/End per frame produces exactly the same simulation as a blocking Update().
//    (Bit-exact comparison of every sphere's position and velocity.)
//----------------------------------------------------------------------//
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJoltAsyncMatchesSynchronousResultTest, "JoltPhysics.Async.MatchesSynchronousResult",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FJoltAsyncMatchesSynchronousResultTest::RunTest(const FString& Parameters)
{
	constexpr int32 NumSteps = 90;

	// Reference: blocking updates.
	TArray<float> Initial;
	TArray<float> Reference;
	{
		FTestPhysicsWorld World(EJobSystemKind::ThreadPool);
		World.BuildSphereBlockScene(8, 5, 8);
		Initial = World.CaptureState();
		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			World.StepSync(FixedDeltaTime, 1);
		}
		Reference = World.CaptureState();
	}

	TestEqual(TEXT("reference has the expected number of values"), Reference.Num(), 8 * 5 * 8 * 6);
	TestTrue(TEXT("the reference simulation actually moved the bodies"),
		Reference.Num() == Initial.Num() && FMemory::Memcmp(Reference.GetData(), Initial.GetData(), Reference.Num() * sizeof(float)) != 0);

	for (const FCombo& Combo : AllCombos)
	{
		const FString Label = ComboLabel(Combo);

		FTestPhysicsWorld World(Combo.JobSystem);
		World.BuildSphereBlockScene(8, 5, 8);
		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			World.StartAsync(FixedDeltaTime, 1, Combo.Launch);
			// (the game thread is free to do other work here)
			World.EndAsync();
			if (World.GetLastError() != JPH::EPhysicsUpdateError::None)
			{
				AddError(FString::Printf(TEXT("%s Update reported an error at step %d"), *Label, Step));
				break;
			}
		}

		const TArray<float> State = World.CaptureState();
		bool bIdentical = State.Num() == Reference.Num();
		int32 FirstDifference = INDEX_NONE;
		for (int32 Index = 0; bIdentical && Index < State.Num(); ++Index)
		{
			if (FMemory::Memcmp(&State[Index], &Reference[Index], sizeof(float)) != 0)
			{
				bIdentical = false;
				FirstDifference = Index;
			}
		}

		if (bIdentical)
		{
			TestTrue(*FString::Printf(TEXT("%s bit-identical to the blocking simulation"), *Label), true);
		}
		else
		{
			AddError(FString::Printf(TEXT("%s differs from the blocking simulation (first difference at value %d: %.9g vs %.9g)"),
				*Label, FirstDifference, FirstDifference == INDEX_NONE ? 0.f : State[FirstDifference], FirstDifference == INDEX_NONE ? 0.f : Reference[FirstDifference]));
		}
	}
	return true;
}

//----------------------------------------------------------------------//
// 4. Informational: how much game-thread work can be hidden behind the step.
//    Never fails on timings, it only reports them (run from the Perf filter).
//----------------------------------------------------------------------//
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJoltAsyncOverlapTimingTest, "JoltPhysics.Async.OverlapTiming",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::PerfFilter)

bool FJoltAsyncOverlapTimingTest::RunTest(const FString& Parameters)
{
	constexpr int32 WarmupSteps = 30;
	constexpr int32 MeasuredSteps = 60;

	for (const FCombo& Combo : AllCombos)
	{
		const FString Label = ComboLabel(Combo);

		// 1) Blocking reference: average cost of one step.
		double SyncStepMs = 0.0;
		{
			FTestPhysicsWorld World(Combo.JobSystem);
			World.BuildSphereBlockScene(12, 10, 12);
			for (int32 Step = 0; Step < WarmupSteps; ++Step)
			{
				World.StepSync(FixedDeltaTime);
			}
			const double T0 = FPlatformTime::Seconds();
			for (int32 Step = 0; Step < MeasuredSteps; ++Step)
			{
				World.StepSync(FixedDeltaTime);
			}
			SyncStepMs = (FPlatformTime::Seconds() - T0) * 1000.0 / MeasuredSteps;
		}

		// 2) Async: the game thread burns as much time as one step costs while the step runs in the background.
		double StartCallMs = 0.0;
		double WaitMs = 0.0;
		double FrameMs = 0.0;
		{
			FTestPhysicsWorld World(Combo.JobSystem);
			World.BuildSphereBlockScene(12, 10, 12);
			for (int32 Step = 0; Step < WarmupSteps; ++Step)
			{
				World.StepSync(FixedDeltaTime);
			}
			for (int32 Step = 0; Step < MeasuredSteps; ++Step)
			{
				const double FrameStart = FPlatformTime::Seconds();
				World.StartAsync(FixedDeltaTime, 1, Combo.Launch);
				const double AfterStart = FPlatformTime::Seconds();
				while ((FPlatformTime::Seconds() - AfterStart) * 1000.0 < SyncStepMs)
				{
					// simulated game-thread work (e.g. other Mass processors)
				}
				const double BeforeWait = FPlatformTime::Seconds();
				World.EndAsync();
				const double FrameEnd = FPlatformTime::Seconds();

				StartCallMs += (AfterStart - FrameStart) * 1000.0;
				WaitMs += (FrameEnd - BeforeWait) * 1000.0;
				FrameMs += (FrameEnd - FrameStart) * 1000.0;
			}
			StartCallMs /= MeasuredSteps;
			WaitMs /= MeasuredSteps;
			FrameMs /= MeasuredSteps;
		}

		AddInfo(FString::Printf(TEXT("%s blocking step: %.3f ms | async: Start() %.3f ms, game-thread work %.3f ms, End() waited %.3f ms, frame %.3f ms (serial would be %.3f ms)"),
			*Label, SyncStepMs, StartCallMs, SyncStepMs, WaitMs, FrameMs, SyncStepMs * 2.0));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
