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

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	//在多个光源离随机一个采样，光源面积作为随机权重
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(const Ray& ray, const std::vector<Object*>& objects, float& tNear, uint32_t& index, Object** hitObject) const
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


Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	Vector3f color(0.0f);
	
	Intersection intersection = intersect(ray);
	if (!intersection.happened) {
		return color;
	}

	if (intersection.m->hasEmission()) {
		return intersection.m->getEmission();
	}


	Vector3f p = intersection.coords;
	Vector3f N = intersection.normal;
	Vector3f wo = -ray.direction;
	// Sample a light source
	Intersection lightSample;
	float pdf_light = 1.0f;
	sampleLight(lightSample, pdf_light);
	Vector3f x = lightSample.coords;
	Vector3f ws = x - p;
	Vector3f NN = lightSample.normal;
	Vector3f emit = lightSample.emit;
	float d2 = dotProduct(ws, ws);
	ws = normalize(ws);

	Ray shadowRay(p + N * EPSILON, ws);
	float d = intersect(shadowRay).distance;

	if (abs(d*d - d2) < EPSILON) {
		Vector3f fr = intersection.m->eval(wo, ws, N);
		float cosTheta =dotProduct(N, ws);
		float cosTheta2 = dotProduct(NN, -ws);
		color += emit * fr * cosTheta * cosTheta2 / (pdf_light * d2);
	}

	if (get_random_float() < RussianRoulette) {
		Vector3f wi = intersection.m->sample(wo, N).normalized();
		Ray newRay(p + N * EPSILON, wi);
		auto isec = intersect(newRay);
		if (isec.happened && isec.m->hasEmission() == false) {
			Vector3f indirectColor = castRay(newRay, depth + 1);
			Vector3f eval = intersection.m->eval(wo, wi, N);
			float cosTheta = dotProduct(N, wi);
			float pdf = intersection.m->pdf(wo, wi, N);
			if (pdf > 0) {
				color += indirectColor *eval  * cosTheta / (pdf * RussianRoulette);
			}
		}
	}

	return color;
}


//Vector3f Scene::castRay(const Ray& ray, int depth) const
//{
//	Vector3f L_dir;
//	Vector3f L_indir;
//
//	Intersection obj_inter = intersect(ray);
//	if (!obj_inter.happened)
//		return L_dir;
//
//	if (obj_inter.m->hasEmission())
//		return obj_inter.m->getEmission();
//
//	Vector3f p = obj_inter.coords;
//	Material* m = obj_inter.m;
//	Vector3f N = obj_inter.normal.normalized();
//	Vector3f wo = ray.direction; 
//
//	float pdf_L = 1.0; 
//	Intersection light_inter;
//	sampleLight(light_inter, pdf_L);    
//
//	Vector3f x = light_inter.coords;
//	Vector3f ws = (x - p).normalized();
//	Vector3f NN = light_inter.normal.normalized();
//	Vector3f emit = light_inter.emit;
//	float d = (x - p).norm();
//
//	Ray Obj2Light(p, ws);
//	float d2 = intersect(Obj2Light).distance;
//	if (d2 - d > -0.001) {
//		Vector3f eval = m->eval(wo, ws, N); 
//		float cos_theta = dotProduct(N, ws);
//		float cos_theta_x = dotProduct(NN, -ws);
//		L_dir = emit * eval * cos_theta * cos_theta_x / std::pow(d, 2) / pdf_L;
//	}
//
//	// L_indir
//	float P_RR = get_random_float();
//	if (P_RR < RussianRoulette) {
//		Vector3f wi = m->sample(wo, N).normalized();
//		Ray r(p, wi);
//		Intersection inter = intersect(r);
//		if (inter.happened && !inter.m->hasEmission()) {
//			Vector3f eval = m->eval(wo, wi, N);
//			float pdf_O = m->pdf(wo, wi, N);
//			float cos_theta = dotProduct(wi, N);
//			L_indir = castRay(r, depth + 1) * eval * cos_theta / pdf_O / RussianRoulette;
//		}
//	}
//	//4->16min
//	return L_dir + L_indir;
//}