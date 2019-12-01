#pragma once
#include <PxPhysicsAPI.h>
#include <iostream>
#include <functional>
#include <vector>

using namespace physx;

class PhysicsEngine
{
private:
	class ErrorLogger : public PxErrorCallback
	{
	public:
		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
		{
			using namespace std;

			switch (code)
			{
			case PxErrorCode::eDEBUG_INFO:
			case PxErrorCode::eNO_ERROR:
				cout << "[Info]";
				break;
			case PxErrorCode::eDEBUG_WARNING:
			case PxErrorCode::ePERF_WARNING:
				cout << "[Warning]";
				break;
			default:
				cout << "[Error]";
				break;
			}
			cout << " : " << message << endl;
			cout << "    " << file << " @ " << line << endl;
		}
	};
	
	static ErrorLogger ErrorCallback;
	static PxDefaultAllocator Allocator;
	PxFoundation  * Foundation = nullptr;
	PxPvd * PVD = nullptr;
	PxPhysics * Physics = nullptr;
	PxCooking * Cooker = nullptr;
	PxScene * GlobalScene = nullptr;
	PxDefaultCpuDispatcher * Dispatcher = nullptr;
	PxMaterial * DefaultMaterial = nullptr;
	PxControllerManager * CharacterManager = nullptr;

	// No need to clean this by hand, they get removed by the sdk along with all the other bodies and stuff
	std::vector<PxTriangleMesh*> TriangleMeshes;
	std::vector<PxController *> Characters;

	PxVec3 Gravity;
	float ElapsedTime;
public:
	PhysicsEngine() = default;
	PhysicsEngine(const PhysicsEngine&) = delete;
	PhysicsEngine& operator=(const PhysicsEngine&) = delete;
	~PhysicsEngine();

	// Initializes the engine
	// Returns true if successful
	// Optionaly you can specify the number of threads to use for simulation and the gravity  acceleration vector
	bool Initialize(uint32_t NumThreads = 2, PxVec3 Gravity = PxVec3(0.0f, -9.81f, 0.0f));

	// Advances to the next step of the simulation
	void Simulate(float ElapsedTimeSeconds);
	
	// Keeps track of the time and simulates only on a fixed frequency (when possible)
	// It accepts an optional lambda that is called just before Simulate is called, which can be used to move stuff
	void SimulateFixedFrequency(float Frequency, std::function<void(float ElapsedTime)> Callback = std::function<void(float)>());

	// Creates a physics triangle mesh from the provided data
	// IMPORTANT : The vertex type (VertexT) MUST have as it's first member(s) 3 floats with the X, Y and Z of the vertex
	template<typename VertexT, typename IndexT = uint32_t>
	size_t CreatePhysicsTriangleMesh(const std::vector<VertexT>& VertexList, const std::vector<IndexT>& IndexList)
	{
		using namespace std;

		if (IndexList.size() % 3 != 0)
		{
			cout << "The index count must be a multiple of 3" << endl;
			return -1;
		}

		PxTriangleMeshDesc mesh_desc;
		mesh_desc.points.count = VertexList.size();
		mesh_desc.points.stride = sizeof(VertexT); // As the position is the very first 3 floats, this works without copy
		mesh_desc.points.data = VertexList.data();

		mesh_desc.triangles.count = IndexList.size() / 3;
		mesh_desc.triangles.stride = 3 * sizeof(IndexT);
		mesh_desc.triangles.data = IndexList.data();

		auto cooked_mesh = Cooker->createTriangleMesh(mesh_desc, Physics->getPhysicsInsertionCallback());
		if (!cooked_mesh)
		{
			cout << "Failed to create the triangle mesh" << endl;
			return -1;
		}

		PxDefaultMemoryOutputStream write_buffer;
		PxTriangleMeshCookingResult::Enum result;
		if (!Cooker->cookTriangleMesh(mesh_desc, write_buffer, &result))
		{
			cout << "Failed to cook the triangle mesh" << endl;
			return -1;
		}

		PxDefaultMemoryInputData read_buffer(write_buffer.getData(), write_buffer.getSize());
		Physics->createTriangleMesh(read_buffer);
		TriangleMeshes.push_back(cooked_mesh);

		return TriangleMeshes.size() - 1;
	}

	// Creates a static actor from a triangle mesh
	bool CreateStaticActor(size_t MeshID, PxVec3 Position, PxQuat Rotation, PxVec3 Scale);
	
	// Creates an heightfield mesh from the pixel data
	// Returns true if successful
	// Heightmap is assumed to be on Row Major order and normalized ([0,1])
	bool CreateTerrain(PxVec3 Position, PxVec3 Scale, uint32_t SizeX, uint32_t SizeY, float MinZ, float MaxZ, const std::vector<float>& Heightmap);

	// Creates a capsule character controller, and returns its ID
	size_t CreateCharacterController(PxVec3 StartPosition, float Height, float Radius);

	// Returns the character controller
	PxController * GetCharacter(size_t ID);

	// Applies the provided displacement to the character, and the gravity (unless ApplyGravity is false)
	// Assumes the provided ID is valid
	PxControllerCollisionFlags MoveCharacter(size_t ID, PxVec3 Disp, float ElapsedTime, bool ApplyGravity = true);
};