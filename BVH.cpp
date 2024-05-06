#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include<float.h>

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives, 0);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, int dim)
{
    BVHBuildNode *node = new BVHBuildNode();
    if(TASK_N<=1) { // we are not using BVH in task 1
        // not building an actual BVH tree, just register the objects
        node->registerObjects(objects);
        return node;
    }
    // TODO: task 2 BVH algorithm starts here
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    if (objects.size() == 1) {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]}, dim);
        node->right = recursiveBuild(std::vector{objects[1]}, dim);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        switch (dim%3) {
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

        node->left = recursiveBuild(leftshapes, dim+1);
        node->right = recursiveBuild(rightshapes, dim+1);

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
    if(TASK_N<=1) {  // we are not using BVH in task 1
        // loop through all objects saved earlier
        std::vector<Object *> objects = node->getRegisteredObjects();
        Intersection first_isect;
        for (int i = 0; i < objects.size(); i++) {
            Intersection isect = objects[i]->getIntersection(ray);
            if (isect.happened && (!first_isect.happened || (first_isect.tnear > isect.tnear))) {
                first_isect = isect;
            }
        }
        return first_isect;
    }
    // TODO: task 2 BVH algorithm starts here
    Intersection inter;
    Vector3f indiv(1.0f/(ray.direction[0]==0?0.00001:ray.direction[0]),
                   1.0f/(ray.direction[1]==0?0.00001:ray.direction[1]),
                   1.0f/(ray.direction[2]==0?0.00001:ray.direction[2]));
       
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0]=int(ray.direction.x>0);
    dirIsNeg[1]=int(ray.direction.y>0);
    dirIsNeg[2]=int(ray.direction.z>0);

    if (!node->bounds.IntersectP(ray, indiv, dirIsNeg))    return inter;

    if (node->left==nullptr && node->right==nullptr)
    {
        inter=node->object->getIntersection(ray);
        return inter;
    }

    Intersection left=getIntersection(node->left, ray);
    Intersection right=getIntersection(node->right, ray);

    return left.tnear < right.tnear ? left : right;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}
