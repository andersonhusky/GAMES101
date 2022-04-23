#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include <float.h>

// 创建bunny的时候会调用一次，场景设置完毕时会调用一次
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives, true);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, bool SAH)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // 最大的为拆分的bounds
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]}, false);
        node->right = recursiveBuild(std::vector{objects[1]}, false);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        // 为包括所有三角形中心的bound
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        // 得到对角线向量中最大的维度
        int dim = centroidBounds.maxExtent();
        float tmin, tmax;
        switch (dim) {
            case 0:
                tmin = centroidBounds.pMin.x;
                tmax = centroidBounds.pMax.x;
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                tmin = centroidBounds.pMin.y;
                tmax = centroidBounds.pMax.y;
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                tmin = centroidBounds.pMin.z;
                tmax = centroidBounds.pMax.z;
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
        }

        std::vector<Object *>::iterator beginning = objects.begin();
        std::vector<Object *>::iterator ending = objects.end();
        std::vector<Object *>::iterator middling = objects.begin();
        int N = 25;
        if(objects.size()<=N)   N = std::max(2, int(objects.size()/2));
        if(SAH){
            std::vector<Bounds3> Buckets(N, Bounds3());
            Bounds3 testbound;
            std::vector<int> BucketsNums(N, 0);
            int objNum = objects.size();
            for(int i=0; i<objNum; i++)
            {
                int k = clamp(0, N-1, int(N*(objects[i]->getBounds().Centroid()[dim]-tmin)/(tmax-tmin)));
                Buckets[k] = Union(Buckets[k], objects[i]->getBounds());
                BucketsNums[k]++;
            }
            // for(int i=0; i<N; i++)  std::cout << BucketsNums[i] << ", ";
            // std::cout << std::endl;
            // for(int i=0; i<N; i++)  std::cout << Buckets[i].pMax << ", " << Buckets[i].pMin << std::endl;

            double BestC_AB=DBL_MAX;
            int bestd=1;
            double S_C = centroidBounds.SurfaceArea();
            for(int i=1; i<N; i++)
            {
                double C_AB = 0.125;
                Bounds3 b0, b1;
                int num0=0, num1=0;
                for(int j=0; j<i; j++)
                {
                    b0 = Union(b0, Buckets[j]);
                    num0 += BucketsNums[j];
                }
                for(int j=i; j<N; j++)
                {
                    b1 = Union(b1, Buckets[j]);
                    num1 += BucketsNums[j];
                }
                C_AB += num0*b0.SurfaceArea()/S_C + num1*b1.SurfaceArea()/S_C;

                if(C_AB<BestC_AB){
                    BestC_AB = C_AB;
                    bestd=i;
                }
            }

            if(BestC_AB<objNum){
                for(int i=0; i<bestd; i++)
                {
                    middling += BucketsNums[i];
                }
            }
            else{
                SAH = false;
                middling += (objects.size() / 2);
            }
        }
        else{
            middling += (objects.size() / 2);
        }

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        // 按照最长的维度划分为左右两组，递归构造
        node->left = recursiveBuild(leftshapes, SAH);
        node->right = recursiveBuild(rightshapes, SAH);

        node->bounds = Union(node->left->bounds, node->right->bounds);
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
    // TODO Traverse the BVH to find intersection
    Intersection inter1, inter2;
    if(!node)   return inter1;

    std::array<int, 3> dirIsNeg;
    if(!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) return inter1;

    if(!node->left && !node->right){
        return node->object->getIntersection(ray);
    }

    inter1 = getIntersection(node->left, ray);
    inter2 = getIntersection(node->right, ray);
    return inter1.distance<inter2.distance? inter1: inter2;
}