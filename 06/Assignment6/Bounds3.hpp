//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
public:
	Vector3f pMin, pMax; // two points to specify the bounding box
	Bounds3()
	{
		double minNum = std::numeric_limits<double>::lowest();
		double maxNum = std::numeric_limits<double>::max();
		pMax = Vector3f(minNum, minNum, minNum);
		pMin = Vector3f(maxNum, maxNum, maxNum);
	}
	Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
	Bounds3(const Vector3f p1, const Vector3f p2)
	{
		pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
		pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
	}
	//对角向量
	Vector3f Diagonal() const { return pMax - pMin; }

	//最大轴方向
	int maxExtent() const
	{
		Vector3f d = Diagonal();
		if (d.x > d.y && d.x > d.z)
			return 0;
		else if (d.y > d.z)
			return 1;
		else
			return 2;
	}

	//表面积
	double SurfaceArea() const
	{
		Vector3f d = Diagonal();
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}

	//中心点
	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

	//交集
	Bounds3 Intersect(const Bounds3& b)
	{
		return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
			fmax(pMin.z, b.pMin.z)),
			Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
				fmin(pMax.z, b.pMax.z)));
	}

	//点在包围盒内的偏移量比例
	Vector3f Offset(const Vector3f& p) const
	{
		Vector3f o = p - pMin;
		if (pMax.x > pMin.x)
			o.x /= pMax.x - pMin.x;
		if (pMax.y > pMin.y)
			o.y /= pMax.y - pMin.y;
		if (pMax.z > pMin.z)
			o.z /= pMax.z - pMin.z;
		return o;
	}

	//是否重叠
	bool Overlaps(const Bounds3& b1, const Bounds3& b2)
	{
		bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
		bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
		bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
		return (x && y && z);
	}

	//判断点是否在包围盒内
	bool Inside(const Vector3f& p, const Bounds3& b)
	{
		return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
			p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
	}

	//0是最小点，1是最大点
	inline const Vector3f& operator[](int i) const
	{
		return (i == 0) ? pMin : pMax;
	}

	//判断射线是否与包围盒相交
	inline bool IntersectP(const Ray& ray) const;
};



inline bool Bounds3::IntersectP(const Ray& ray) const
{
	Vector3f invDir = ray.direction_inv;
	auto t1 = (pMin - ray.origin) * invDir;
	auto t2 = (pMax - ray.origin) * invDir;
	auto tmin = Vector3f::Min(t1, t2);
	auto tmax = Vector3f::Max(t1, t2);
	float tNear = std::max(std::max(tmin.x, tmin.y), tmin.z);
	float tFar = std::min(std::min(tmax.x, tmax.y), tmax.z);
	//如果tNear大于tFar说明射线和包围盒没有相交
	return tNear<tFar && tFar>0;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
	ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
	return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b.pMin, p);
	ret.pMax = Vector3f::Max(b.pMax, p);
	return ret;
}

#endif // RAYTRACING_BOUNDS3_H
