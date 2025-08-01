#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
	SplitMethod splitMethod)
	: maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
	primitives(std::move(p))
{
	time_t start, stop;
	time(&start);
	if (primitives.empty())
		return;

	root = recursiveBuild(primitives);

	time(&stop);
	double diff = difftime(stop, start);
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	int secs = (int)diff - (hrs * 3600) - (mins * 60);

	printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
		hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	for (int i = 0; i < objects.size(); ++i)
		bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		node->area = objects[0]->getArea();
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		node->area = node->left->area + node->right->area;
		return node;
	}
	else {
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();
		switch (dim) {
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().x <
					f2->getBounds().Centroid().x;
				});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().y <
					f2->getBounds().Centroid().y;
				});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().z <
					f2->getBounds().Centroid().z;
				});
			break;
		}

		auto beginning = objects.begin();
		auto middling = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		assert(objects.size() == (leftshapes.size() + rightshapes.size()));

		node->left = recursiveBuild(leftshapes);
		node->right = recursiveBuild(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);
		node->area = node->left->area + node->right->area;
	}

	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
	Intersection isect;
	if (!root)
		return isect;
	isect = BVHAccel::getIntersection(root, ray);
	return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
	Intersection isec;
	if (node == nullptr || node->bounds.IntersectP(ray) == false) return isec;
	if (node->object != nullptr) {
		isec = node->object->getIntersection(ray);
		return isec;
	}
	Intersection leftIsect = getIntersection(node->left, ray);
	Intersection rightIsect = getIntersection(node->right, ray);
	if (leftIsect.happened && rightIsect.happened) {
		if (leftIsect.distance < rightIsect.distance) {
			isec = leftIsect;
		}
		else {
			isec = rightIsect;
		}
	}
	else if (leftIsect.happened) {
		isec = leftIsect;
	}
	else if (rightIsect.happened) {
		isec = rightIsect;
	}
	return isec;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection& pos, float& pdf) {
	if (node->left == nullptr || node->right == nullptr) {
		node->object->Sample(pos, pdf);
		pdf *= node->area; //好像还是1
		return;
	}
	if (p < node->left->area) getSample(node->left, p, pos, pdf);
	else getSample(node->right, p - node->left->area, pos, pdf);
}

//在BVH树上采样一个点，并返回该点的概率密度
void BVHAccel::Sample(Intersection& pos, float& pdf) {
	float p = std::sqrt(get_random_float()) * root->area;
	getSample(root, p, pos, pdf);
	pdf /= root->area;//面积分之一
}