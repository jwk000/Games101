//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
	return this->bvh->Intersect(ray);
}


bool Scene::trace(const Ray& ray, const std::vector<Object*>& objects, float& tNear, uint32_t& index, Object** hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear = tNearK;
			index = indexK;
		}
	}


	return (*hitObject != nullptr);
}

// Implementation of the Whitted-syle light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refracton direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refractin depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is duffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	if (depth > this->maxDepth) {
		return Vector3f(0.0, 0.0, 0.0);
	}
	Intersection intersection = Scene::intersect(ray);
	Material* m = intersection.m;
	Object* hitObject = intersection.obj;
	Vector3f hitColor = this->backgroundColor;
	Vector2f uv;
	uint32_t index = 0;
	if (intersection.happened) {

		Vector3f hitPoint = intersection.coords;
		Vector3f N = intersection.normal; // normal
		Vector2f st; // st coordinates
		hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
		switch (m->getType()) {
		case REFLECTION_AND_REFRACTION:
		{
			Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
			Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));
			Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
				hitPoint - N * EPSILON :
				hitPoint + N * EPSILON;
			Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
				hitPoint - N * EPSILON :
				hitPoint + N * EPSILON;
			Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
			Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
			float kr;
			fresnel(ray.direction, N, m->ior, kr);
			hitColor = reflectionColor * kr + refractionColor * (1 - kr);
			break;
		}
		case REFLECTION:
		{
			float kr;
			fresnel(ray.direction, N, m->ior, kr);
			Vector3f reflectionDirection = reflect(ray.direction, N);
			Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?hitPoint + N * EPSILON :hitPoint - N * EPSILON;
			hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
			break;
		}
		default:
		{
			Vector3f lightAmt = 0, specularColor = 0;
			//增加一个偏移，避免射线和物体表面相交 
			Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?hitPoint + N * EPSILON : hitPoint - N * EPSILON;
			for (uint32_t i = 0; i < get_lights().size(); ++i)
			{
				Vector3f lightDir = get_lights()[i]->position - hitPoint;
				// square of the distance between hitPoint and the light
				//光源到hitPoint距离的平方
				float lightDistance2 = dotProduct(lightDir, lightDir);
				lightDir = normalize(lightDir);
				float LdotN = std::max(0.f, dotProduct(lightDir, N));
				Object* shadowHitObject = nullptr;
				float tNearShadow = kInfinity;
				// is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
				//从hitPoint到光源打一个射线如果被阻挡了说明hitPoint在阴影里
				bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
				lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN;
				Vector3f reflectionDirection = reflect(-lightDir, N);//这里用的是反射方向和视线方向的余弦，没用半程向量
				specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
					m->specularExponent) * get_lights()[i]->intensity;

			}
			hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
			break;
		}
		}
	}

	return hitColor;
}