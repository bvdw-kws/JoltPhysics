// Copyright (C) 2024 Van de Walle Bastien
// SPDX-License-Identifier: MIT

#pragma once

// UE headers first, then Jolt (same order as JPRPhysicsSubsystem.cpp).
#include "CoreMinimal.h"
#include "Async/Async.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformTLS.h"

#include <atomic>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/JobSystemUETask.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysicsTests
{
	namespace Layers
	{
		constexpr JPH::ObjectLayer NonMoving = 0;
		constexpr JPH::ObjectLayer Moving = 1;
	}

	namespace BroadPhaseLayers
	{
		constexpr JPH::BroadPhaseLayer NonMoving(0);
		constexpr JPH::BroadPhaseLayer Moving(1);
		constexpr JPH::uint Count = 2;
	}

	/** Which Jolt job system drives the simulation. */
	enum class EJobSystemKind : uint8
	{
		ThreadPool,	// JPH::JobSystemThreadPool: Jolt owns N worker threads (what UJPRPhysicsSubsystem uses today)
		UETask,		// JPH::JobSystemUETask: each Jolt job is a UE::Tasks task on the engine worker pool
	};

	/** How PhysicsSystem::Update is moved off the calling thread. */
	enum class EAsyncLaunch : uint8
	{
		DedicatedThread,	// EAsyncExecution::Thread: a thread owned by the step, never starves the task graph
		TaskGraph,			// EAsyncExecution::TaskGraph: the blocking Update() occupies one engine worker
	};

	inline const TCHAR* ToString(const EJobSystemKind Kind)
	{
		return Kind == EJobSystemKind::ThreadPool ? TEXT("JobSystemThreadPool") : TEXT("JobSystemUETask");
	}

	inline const TCHAR* ToString(const EAsyncLaunch Launch)
	{
		return Launch == EAsyncLaunch::DedicatedThread ? TEXT("DedicatedThread") : TEXT("TaskGraph");
	}

	class FObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
	{
	public:
		virtual bool ShouldCollide(JPH::ObjectLayer InObject1, JPH::ObjectLayer InObject2) const override
		{
			switch (InObject1)
			{
			case Layers::NonMoving: return InObject2 == Layers::Moving;
			case Layers::Moving: return true;
			default: return false;
			}
		}
	};

	class FBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface
	{
	public:
		virtual JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::Count; }

		virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer InLayer) const override
		{
			return InLayer == Layers::NonMoving ? BroadPhaseLayers::NonMoving : BroadPhaseLayers::Moving;
		}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
		virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer InLayer) const override
		{
			return InLayer == BroadPhaseLayers::NonMoving ? "NonMoving" : "Moving";
		}
#endif
	};

	class FObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
	{
	public:
		virtual bool ShouldCollide(JPH::ObjectLayer InLayer1, JPH::BroadPhaseLayer InLayer2) const override
		{
			switch (InLayer1)
			{
			case Layers::NonMoving: return InLayer2 == BroadPhaseLayers::Moving;
			case Layers::Moving: return true;
			default: return false;
			}
		}
	};

	/**
	 * Contact listener that can park the simulation inside OnContactAdded until the test opens the gate.
	 * This gives a deterministic way to know that PhysicsSystem::Update is "in flight" (no timing involved).
	 */
	class FGatedContactListener final : public JPH::ContactListener
	{
	public:
		static constexpr uint32 GateTimeoutMs = 10000;

		FGatedContactListener() : Gate(FPlatformProcess::GetSynchEventFromPool(true)) {}
		virtual ~FGatedContactListener() override { FPlatformProcess::ReturnSynchEventToPool(Gate); }

		virtual void OnContactAdded(const JPH::Body& InBody1, const JPH::Body& InBody2,
			const JPH::ContactManifold& InManifold, JPH::ContactSettings& IoSettings) override
		{
			ContactThreadId.store(FPlatformTLS::GetCurrentThreadId());
			bContactSeen.store(true);
			if (bBlockOnGate && !Gate->Wait(GateTimeoutMs))
			{
				bGateTimedOut.store(true);
			}
		}

		void OpenGate() { Gate->Trigger(); }

		bool bBlockOnGate = true;
		std::atomic<bool> bContactSeen{ false };
		std::atomic<bool> bGateTimedOut{ false };
		std::atomic<uint32> ContactThreadId{ 0 };

	private:
		FEvent* Gate = nullptr;
	};

	/**
	 * A standalone Jolt world built the same way UJPRPhysicsSubsystem::CreatePhysicsSystem builds it
	 * (factory + types, temp allocator, job system, layer interfaces, PhysicsSystem::Init), but without
	 * needing a UWorld or a physics layer data asset.
	 */
	class FTestPhysicsWorld
	{
	public:
		explicit FTestPhysicsWorld(const EJobSystemKind InKind, const int32 InNumThreads = 4)
			: Kind(InKind)
		{
			JPH::RegisterDefaultAllocator();
			// The factory is a process-wide singleton. Only tear it down if we created it, so a running
			// editor world's UJPRPhysicsSubsystem is never affected.
			if (JPH::Factory::sInstance == nullptr)
			{
				JPH::Factory::sInstance = new JPH::Factory();
				JPH::RegisterTypes();
				bOwnsFactory = true;
			}

			TempAllocator = MakeShareable<JPH::TempAllocator>(new JPH::TempAllocatorImpl(32 * 1024 * 1024));
			if (Kind == EJobSystemKind::ThreadPool)
			{
				JobSystem = MakeShared<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, InNumThreads);
			}
			else
			{
				JobSystem = MakeShared<JPH::JobSystemUETask>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers);
			}

			BroadPhaseLayerInterface = MakeShared<FBroadPhaseLayerInterface>();
			ObjectVsBroadPhaseLayerFilter = MakeShared<FObjectVsBroadPhaseLayerFilter>();
			ObjectLayerPairFilter = MakeShared<FObjectLayerPairFilter>();

			PhysicsSystem = MakeShared<JPH::PhysicsSystem>();
			PhysicsSystem->Init(MaxBodies, /*NumBodyMutexes*/ 0, MaxBodyPairs, MaxContactConstraints,
				*BroadPhaseLayerInterface, *ObjectVsBroadPhaseLayerFilter, *ObjectLayerPairFilter);
		}

		~FTestPhysicsWorld()
		{
			EndAsync();
			if (PhysicsSystem.IsValid())
			{
				PhysicsSystem->SetContactListener(nullptr);
			}
			PhysicsSystem.Reset();
			ObjectLayerPairFilter.Reset();
			ObjectVsBroadPhaseLayerFilter.Reset();
			BroadPhaseLayerInterface.Reset();
			JobSystem.Reset();
			TempAllocator.Reset();

			if (bOwnsFactory && JPH::Factory::sInstance != nullptr)
			{
				JPH::UnregisterTypes();
				delete JPH::Factory::sInstance;
				JPH::Factory::sInstance = nullptr;
			}
		}

		FTestPhysicsWorld(const FTestPhysicsWorld&) = delete;
		FTestPhysicsWorld& operator=(const FTestPhysicsWorld&) = delete;

		JPH::BodyInterface& Bodies() const { return PhysicsSystem->GetBodyInterface(); }

		// ---- Scene -------------------------------------------------------------------------------------

		void AddFloor()
		{
			JPH::BodyCreationSettings Settings(new JPH::BoxShape(JPH::Vec3(500.f, 1.f, 500.f)), JPH::RVec3(0, -1, 0),
				JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NonMoving);
			Bodies().CreateAndAddBody(Settings, JPH::EActivation::DontActivate);
		}

		JPH::BodyID AddSphere(const float X, const float Y, const float Z)
		{
			JPH::BodyCreationSettings Settings(new JPH::SphereShape(0.5f), JPH::RVec3(X, Y, Z),
				JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::Moving);
			const JPH::BodyID Id = Bodies().CreateAndAddBody(Settings, JPH::EActivation::Activate);
			SphereIds.Add(Id);
			return Id;
		}

		/** A floor plus one sphere that already penetrates it, so the very first collision step reports a contact. */
		void BuildSingleContactScene()
		{
			AddFloor();
			AddSphere(0.f, 0.4f, 0.f);
			PhysicsSystem->OptimizeBroadPhase();
		}

		/** A floor plus a block of spheres dropped from slightly different heights: many contacts, many jobs. */
		void BuildSphereBlockScene(const int32 CountX, const int32 CountY, const int32 CountZ)
		{
			AddFloor();
			for (int32 Y = 0; Y < CountY; ++Y)
			{
				for (int32 Z = 0; Z < CountZ; ++Z)
				{
					for (int32 X = 0; X < CountX; ++X)
					{
						// Slight x/z jitter derived from the index (no RNG) so spheres topple instead of stacking perfectly.
						const float Jitter = 0.01f * static_cast<float>((X * 7 + Y * 13 + Z * 29) % 11);
						AddSphere(X * 1.1f + Jitter, 0.4f + Y * 1.05f, Z * 1.1f - Jitter);
					}
				}
			}
			PhysicsSystem->OptimizeBroadPhase();
		}

		// ---- Stepping ----------------------------------------------------------------------------------

		/** Blocking step on the calling thread. */
		JPH::EPhysicsUpdateError StepSync(const float DeltaTime, const int32 CollisionSteps = 1)
		{
			LastError = PhysicsSystem->Update(DeltaTime, CollisionSteps, TempAllocator.Get(), JobSystem.Get());
			return LastError;
		}

		/** Non-blocking step: PhysicsSystem::Update runs on another thread, this returns right away. */
		void StartAsync(const float DeltaTime, const int32 CollisionSteps, const EAsyncLaunch Launch)
		{
			check(!Pending.IsValid());
			bStepFinished.store(false);
			auto Work = [this, DeltaTime, CollisionSteps]()
			{
				LastError = PhysicsSystem->Update(DeltaTime, CollisionSteps, TempAllocator.Get(), JobSystem.Get());
				bStepFinished.store(true);
			};
			Pending = Async(Launch == EAsyncLaunch::DedicatedThread ? EAsyncExecution::Thread : EAsyncExecution::TaskGraph, MoveTemp(Work));
		}

		/** Equivalent of ForceEndPhysicsSimulation(): blocks until the asynchronous step is done. */
		void EndAsync()
		{
			if (Pending.IsValid())
			{
				Pending.Wait();
				Pending = TFuture<void>();
			}
		}

		bool IsAsyncStepRunning() const { return Pending.IsValid() && !Pending.IsReady(); }
		bool IsStepFinished() const { return bStepFinished.load(); }
		JPH::EPhysicsUpdateError GetLastError() const { return LastError; }

		// ---- Inspection --------------------------------------------------------------------------------

		/** Position + linear velocity of every sphere, as raw floats, for bit-exact comparisons. */
		TArray<float> CaptureState() const
		{
			TArray<float> State;
			State.Reserve(SphereIds.Num() * 6);
			for (const JPH::BodyID Id : SphereIds)
			{
				const JPH::RVec3 Position = Bodies().GetCenterOfMassPosition(Id);
				const JPH::Vec3 Velocity = Bodies().GetLinearVelocity(Id);
				State.Add(static_cast<float>(Position.GetX()));
				State.Add(static_cast<float>(Position.GetY()));
				State.Add(static_cast<float>(Position.GetZ()));
				State.Add(Velocity.GetX());
				State.Add(Velocity.GetY());
				State.Add(Velocity.GetZ());
			}
			return State;
		}

		EJobSystemKind GetKind() const { return Kind; }

		TSharedPtr<JPH::PhysicsSystem> PhysicsSystem;
		TArray<JPH::BodyID> SphereIds;

	private:
		static constexpr JPH::uint MaxBodies = 8192;
		static constexpr JPH::uint MaxBodyPairs = 32768;
		static constexpr JPH::uint MaxContactConstraints = 16384;

		EJobSystemKind Kind;
		bool bOwnsFactory = false;
		JPH::EPhysicsUpdateError LastError = JPH::EPhysicsUpdateError::None;
		std::atomic<bool> bStepFinished{ false };
		TFuture<void> Pending;

		TSharedPtr<JPH::TempAllocator> TempAllocator;
		TSharedPtr<JPH::JobSystem> JobSystem;
		TSharedPtr<JPH::BroadPhaseLayerInterface> BroadPhaseLayerInterface;
		TSharedPtr<JPH::ObjectVsBroadPhaseLayerFilter> ObjectVsBroadPhaseLayerFilter;
		TSharedPtr<JPH::ObjectLayerPairFilter> ObjectLayerPairFilter;
	};
}
