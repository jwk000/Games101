//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, Microfacet};

class Material {
private:

	// Compute reflection direction
	Vector3f reflect(const Vector3f& I, const Vector3f& N) const
	{
		return I - 2 * dotProduct(I, N) * N;
	}

	// Compute refraction direction using Snell's law
	// We need to handle with care the two possible situations:
	//    - When the ray is inside the object
	//    - When the ray is outside.
	// If the ray is outside, you need to make cosi positive cosi = -N.I
	// If the ray is inside, you need to invert the refractive indices and negate the normal N
	Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
	{
		float cosi = clamp(-1, 1, dotProduct(I, N));
		float etai = 1, etat = ior;
		Vector3f n = N;
		if (cosi < 0) { cosi = -cosi; }
		else { std::swap(etai, etat); n = -N; }
		float eta = etai / etat;
		float k = 1 - eta * eta * (1 - cosi * cosi);
		return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
	}

	// Compute Fresnel equation
	// \param I is the incident view direction
	// \param N is the normal at the intersection point
	// \param ior is the material refractive index
	// \param[out] kr is the amount of light reflected
	void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
	{
		float cosi = clamp(-1, 1, dotProduct(I, N));
		float etai = 1, etat = ior;
		if (cosi > 0) { std::swap(etai, etat); }
		// Compute sini using Snell's law
		float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
		// Total internal reflection
		if (sint >= 1) {
			kr = 1;
		}
		else {
			float cost = sqrtf(std::max(0.f, 1 - sint * sint));
			cosi = fabsf(cosi);
			float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
			float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
			kr = (Rs * Rs + Rp * Rp) / 2;
		}
		// As a consequence of the conservation of energy, transmittance is given by:
		// kt = 1 - kr;
	}

	//a是局部坐标系下的向量，需要把a转换成世界坐标系才能用于计算
	//N是世界空间下的法线，C是N在世界坐标系xz平面或yz屏幕的投影，BCN构成一组基向量

	Vector3f toWorld(const Vector3f& a, const Vector3f& N) {
		Vector3f B, C;
		if (std::fabs(N.x) > std::fabs(N.y)) {
			float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
			C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
		}
		else {
			float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
			C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
		}
		B = crossProduct(C, N);
		return a.x * B + a.y * C + a.z * N;
	}

public:
	MaterialType m_type;
	Vector3f m_emission;
	float ior;
	Vector3f Kd, Ks;
	float specularExponent;

	inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
	inline MaterialType getType();
	inline Vector3f getColorAt(double u, double v);
	inline Vector3f getEmission();
	inline bool hasEmission();

	// sample a ray by Material properties
	inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
	// given a ray, calculate the PdF of this ray
	inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
	// given a ray, calculate the contribution of this ray
	inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);

private:
	float DistributionGGX(Vector3f N, Vector3f H, float roughness)
	{
		float a = roughness * roughness;
		float a2 = a * a;
		float NdotH = std::max(dotProduct(N, H), 0.0f);
		float NdotH2 = NdotH * NdotH;

		float nom = a2;
		float denom = (NdotH2 * (a2 - 1.0) + 1.0);
		denom = M_PI * denom * denom;

		return nom / std::max(denom, 0.0000001f); // prevent divide by zero for roughness=0.0 and NdotH=1.0
	}

	float GeometrySchlickGGX(float NdotV, float k)
	{
		float nom = NdotV;
		float denom = NdotV * (1.0 - k) + k;
		return nom / denom;
	}

	float GeometrySmith(Vector3f N, Vector3f V, Vector3f L, float roughness)
	{
		float r = (roughness + 1.0);
		float k = (r * r) / 8.0;
		float NdotV = std::max(dotProduct(N, V), 0.0f);
		float NdotL = std::max(dotProduct(N, L), 0.0f);
		float ggx2 = GeometrySchlickGGX(NdotV, k);
		float ggx1 = GeometrySchlickGGX(NdotL, k);

		return ggx1 * ggx2;
	}


};

Material::Material(MaterialType t, Vector3f e) {
	m_type = t;
	m_emission = e;
}

MaterialType Material::getType() { return m_type; }
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission() {
	if (m_emission.norm() > EPSILON) return true;
	else return false;
}

Vector3f Material::getColorAt(double u, double v) {
	return Vector3f();
}

// 给定入射方向与法向量，随机采样一个出射方向
// 在半径是1的半球上均匀采样一个坐标（diffuse和wi无关）
// 先随机一个高度z，计算投影半径r，再随机一个角度phi
// 得到一个局部坐标系下的向量localRay，转到世界坐标系
Vector3f Material::sample(const Vector3f& wi, const Vector3f& N) {
	switch (m_type) {
	case DIFFUSE:
	case Microfacet:
	{
		// uniform sample on the hemisphere
		float x_1 = get_random_float(), x_2 = get_random_float();
		float z = std::fabs(1.0f - 2.0f * x_1);//-1,1
		float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
		return toWorld(localRay, N);

		break;
	}
	}
}

//给定一对入射、出射方向与法向量，计算 sample 方法得到该出射方向的概率密度
//diffuse在半球上的分布概率是个常数 1/2pi
float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
	switch (m_type) {
	case DIFFUSE:
	case Microfacet:
	{
		// uniform sample probability 
		if (dotProduct(wo, N) > 0.0f)
			return 0.5f / M_PI;
		else
			return 0.0f;
		break;
	}
	}
}

// 给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值
// diffuse的fr是固定值 kd/pi
Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
	switch (m_type) {
	case DIFFUSE:
	{
		// calculate the contribution of diffuse   model
		float cosalpha = dotProduct(N, wo);
		if (cosalpha > 0.0f) {
			Vector3f diffuse = Kd / M_PI;
			return diffuse;
		}
		else
			return Vector3f(0.0f);
		break;
	}
	case Microfacet://微表面材质的BRDF
	{
		// Disney PBR 方案
		float cosalpha = dotProduct(N, wo);
		if (cosalpha > 0.0f) {
			float roughness = 0.40;

			Vector3f V = -wi;
			Vector3f L = wo;
			Vector3f H = normalize(V + L);

			// 计算 distribution of normals: D
			float D = DistributionGGX(N, H, roughness);

			// 计算 shadowing masking term: G
			float G = GeometrySmith(N, V, L, roughness);

			// 计算 fresnel 系数: F
			float F;
			float etat = 1.85;
			fresnel(wi, N, etat, F);

			Vector3f nominator = D * G * F;
			float denominator = 4 * std::max(dotProduct(N, V), 0.0f) * std::max(dotProduct(N, L), 0.0f);
			Vector3f specular = nominator / std::max(denominator, 0.001f);

			// 能量守恒
			float ks_ = F;//反射比率
			float kd_ = 1.0f - ks_;//折射比率

			Vector3f diffuse = 1.0f / M_PI;

			return Ks * specular +  Kd * diffuse;
		}
		else
			return Vector3f(0.0f);
		break;
	}
	}
}


#endif //RAYTRACING_MATERIAL_H
