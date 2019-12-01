#include "PhysicsEngine.h"
#include <iostream>
#include <chrono>

using namespace std;

int main()
{
	PhysicsEngine physics;

	physics.Initialize();

	// Create character
	auto character_id = physics.CreateCharacterController(PxVec3(0,0,0), 125, 20);
	auto character_ptr = physics.GetCharacter(character_id);

	// Create a few simple meshes
	{
		vector<PxVec3> vertices =
		{
			PxVec3(-1, 0, -1),
			PxVec3(-1, 0,  1),
			PxVec3( 1, 0, -1),
			PxVec3( 1, 0,  1)
		};

		vector<uint32_t> indices =
		{
			3, 2, 0,
			3, 0, 1
		};

		auto mesh_id = physics.CreatePhysicsTriangleMesh(vertices, indices);
		
		physics.CreateStaticActor(mesh_id, PxVec3(0, -250, 0), PxQuat(PxIdentity), PxVec3(500, 1, 500));
		physics.CreateStaticActor(mesh_id, PxVec3(100, -400, 0), PxQuat(PxIdentity), PxVec3(800, 1, 800));
	}

	// Game loop
	while(true)
	{
		// Simulate physics with a fixed 60 Hz loop
		// Use the callback as the place to update stuff (like moving characters)
		physics.SimulateFixedFrequency(60, [&](float ElapsedTime)
		{
			physics.MoveCharacter(character_id, PxVec3(7, 0, 0), ElapsedTime);
		});

		// Do your rendering here
		cout << character_ptr->getPosition().x << " " << character_ptr->getPosition().y << " " << character_ptr->getPosition().z << endl;
	}

	return 0;
}