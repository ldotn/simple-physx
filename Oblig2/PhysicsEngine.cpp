#include "PhysicsEngine.h"
#include <chrono>

PhysicsEngine::ErrorLogger PhysicsEngine::ErrorCallback;
PxDefaultAllocator PhysicsEngine::Allocator;

PhysicsEngine::~PhysicsEngine()
{
	if (GlobalScene)
		GlobalScene->release();
	if (Dispatcher)
		Dispatcher->release();
	if (Physics)
		Physics->release();

	if (PVD)
	{
		PxPvdTransport* transport = PVD->getTransport();
		PVD->release();
		transport->release();
	}
	
	if (Foundation)
		Foundation->release();
}

bool PhysicsEngine::Initialize(uint32_t NumThreads, PxVec3 Gravity)
{
	this->Gravity = Gravity;

	using namespace std;

	Foundation = PxCreateFoundation(PX_FOUNDATION_VERSION, Allocator, ErrorCallback);
	if (!Foundation)
	{
		cout << "Failed to create the PhysX foundation instance" << endl;
		return false;
	}

#ifdef _DEBUG
	bool record_memory_allocations = true;

	// Support for the PhysX Visual Debugger [https://developer.nvidia.com/physx-visual-debugger]
	PVD = PxCreatePvd(*Foundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	if (!PVD->connect(*transport, PxPvdInstrumentationFlag::eALL))
		cout << "[Warning] Could not connect to the visual debugger. Maybe it's not open?" << endl;
#else
	bool record_memory_allocations = false;
#endif

	PxTolerancesScale scaling;
	scaling.length = 100;

	Physics = PxCreatePhysics(PX_PHYSICS_VERSION, *Foundation, scaling, record_memory_allocations, PVD);
	if (!Physics)
	{
		cout << "Failed to create the PhysX physics instance" << endl;
		return false;
	}

	Cooker = PxCreateCooking(PX_PHYSICS_VERSION, *Foundation, PxCookingParams(scaling));
	if (!Cooker)
	{
		cout << "Failed to create the PhysX cooker instance" << endl;
		return false;
	}
	
	PxSceneDesc scene_desc(Physics->getTolerancesScale());
	scene_desc.gravity = Gravity;
	Dispatcher = PxDefaultCpuDispatcherCreate(NumThreads);
	scene_desc.cpuDispatcher = Dispatcher;
	scene_desc.filterShader = PxDefaultSimulationFilterShader;

	GlobalScene = Physics->createScene(scene_desc);
	
	CharacterManager = PxCreateControllerManager(*GlobalScene);

#ifdef _DEBUG
	PxPvdSceneClient* pvd_client = GlobalScene->getScenePvdClient();
	if (pvd_client)
	{
		pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
#endif

	// Create default material
	DefaultMaterial = Physics->createMaterial(0.5f, 0.5f, 0.6f);

	return true;
}

void PhysicsEngine::Simulate(float ElapsedTimeSeconds)
{
	GlobalScene->simulate(ElapsedTimeSeconds);
	GlobalScene->fetchResults(true);
}

bool PhysicsEngine::CreateStaticActor(size_t MeshID,PxVec3 Position, PxQuat Rotation, PxVec3 Scale)
{
	using namespace std;

	if (MeshID < 0 && MeshID >= TriangleMeshes.size())
	{
		cout << "Invalid mesh id [" << MeshID << "] provided to CreateStaticActor" << endl;
		return false;
	}

	PxTriangleMeshGeometry instance;
	instance.triangleMesh = TriangleMeshes[MeshID];
	instance.scale = Scale;
	
	PxTransform tworld(Position, Rotation);

	// No need to keep the rigid body pointer and clean it by hand, they are removed by the SDK
	auto actor = PxCreateStatic(*Physics, tworld, instance, *DefaultMaterial);
	if (!actor)
	{
		cout << "Failed to create the static actor" << endl;
		return false;
	}
	GlobalScene->addActor(*actor);

	return true;
}

bool PhysicsEngine::CreateTerrain(PxVec3 Position, PxVec3 Scale, uint32_t SizeX, uint32_t SizeY, float MinZ, float MaxZ, const std::vector<float>& Heightmap)
{
	using namespace std;

	PxHeightFieldDesc hf_desc;
	hf_desc.format = PxHeightFieldFormat::eS16_TM;
	hf_desc.nbColumns = SizeX;
	hf_desc.nbRows = SizeY;

	vector<PxHeightFieldSample> px_data(Heightmap.size());

	for (uint32_t y = 0; y < SizeY; y++)
	{
		for (uint32_t x = 0; x < SizeX; x++)
		{
			uint32_t out_idx = y + x * hf_desc.nbRows;
			uint32_t in_idx = x * sizeof(float) + y * SizeX;

			px_data[out_idx].height = (MaxZ - MinZ) * Heightmap[in_idx] + MinZ;
			px_data[out_idx].materialIndex0 = 0;
			px_data[out_idx].materialIndex1 = 0;
		}
	}

	hf_desc.samples.data = px_data.data();
	hf_desc.samples.stride = sizeof(PxHeightFieldSample);

	auto hf_ptr = Cooker->createHeightField(hf_desc, Physics->getPhysicsInsertionCallback());
	PxHeightFieldGeometry geo(hf_ptr, PxMeshGeometryFlags(), 1, Scale.x / hf_desc.nbColumns, Scale.z / hf_desc.nbRows);

	PxTransform tworld(PxVec3(Position.x, Position.y, Position.z));

	// No need to keep the rigid body pointer and clean it by hand, they are removed by the SDK
	auto actor = PxCreateStatic(*Physics, tworld, geo, *DefaultMaterial);
	if (!actor)
	{
		cout << "Failed to create the terrain actor" << endl;
		return false;
	}
	GlobalScene->addActor(*actor);

	return true;
}

size_t PhysicsEngine::CreateCharacterController(PxVec3 StartPosition, float Height, float Radius)
{
	PxCapsuleControllerDesc desc;

	desc.height = Height;
	desc.radius = Radius;
	desc.position = PxExtendedVec3(StartPosition.x, StartPosition.y, StartPosition.z);
	desc.contactOffset = Radius + Radius * 0.1;
	desc.stepOffset = Height * 0.25;
	desc.material = DefaultMaterial;

	PxController * controller = CharacterManager->createController(desc);
	if (!controller)
	{
		std::cout << "Failed to create the character controller" << std::endl;
		return -1;
	}
	Characters.push_back(controller);

	return Characters.size() - 1;
}

PxController * PhysicsEngine::GetCharacter(size_t ID)
{
	if (ID < 0 && ID >= Characters.size())
	{
		std::cout << "Invalid character ID [" << ID << "] provided to GetCharacter" << std::endl;
		return nullptr;
	}

	return Characters[ID];
}

void PhysicsEngine::SimulateFixedFrequency(float Frequency, std::function<void(float ElapsedTime)> Callback)
{
	using namespace std;

	static auto start_time = chrono::high_resolution_clock::now();
	float step_size = 1.0f / Frequency;

	ElapsedTime = chrono::duration<float, std::deca>(chrono::high_resolution_clock::now() - start_time).count();
	if (ElapsedTime >= step_size)
	{
		if (Callback) Callback(ElapsedTime);

		Simulate(ElapsedTime);
		start_time = chrono::high_resolution_clock::now();
	}
}

PxControllerCollisionFlags PhysicsEngine::MoveCharacter(size_t ID, PxVec3 Disp, float ElapsedTime, bool ApplyGravity)
{
	auto char_ptr = GetCharacter(ID);
	if (ApplyGravity) Disp += Gravity;

	return char_ptr->move(Disp, 1e-6, ElapsedTime, PxControllerFilters());
}
