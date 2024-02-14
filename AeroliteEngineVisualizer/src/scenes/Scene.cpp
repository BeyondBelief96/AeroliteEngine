#include "Scene.h"

void Scene::Destroy()
{
	running = false;
	world->ClearWorld();
}

void Scene::UpdatePhysicsProperties(const real friction, const real restitution) const
{
	for (const auto& body : world->GetBodies())
	{
		body->SetFriction(friction);
		body->SetRestitution(restitution);
	}
}
