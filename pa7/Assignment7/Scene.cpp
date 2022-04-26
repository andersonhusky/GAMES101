//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene:: sampleLight(Intersection &pos, float &pdf) const
{
    // 所有的光源面积之和
    float emit_area_sum = 0;
    // 遍历obj，累加是光源的物体的面积
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // 在面积随机采样一个数值
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            // 确定采样到在哪个光源
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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

// Implementation of Path Tracing
Vector3f Scene::shade(const Intersection &intersection, const Vector3f &obj2ViewDir) const
{
    // TO DO Implement Path Tracing Algorithm here
    if(intersection.m->hasEmission())   return intersection.m->getEmission();

    const float epsilon = 0.0005f;
    Vector3f hitPoint = intersection.coords;
    Vector3f N = intersection.normal;

    Vector3f L_dir(0.f), L_indir(0.f);
    // -------------light part-------------------
    Intersection inter;
    float pdf_light;
    sampleLight(inter, pdf_light);
    // 随机的光源面上的点、光源到反射点的向量、入射方向、光源面法向量、亮度
    Vector3f hitLightPoint = inter.coords;
    Vector3f obj2Light = hitLightPoint-hitPoint;
    Vector3f obj2LightDir = normalize(obj2Light);
    Vector3f NN = inter.normal;
    Vector3f emit = inter.emit;

    Intersection interTest = intersect(Ray(hitPoint, obj2LightDir));
    if(obj2Light.norm() - interTest.distance < epsilon){
        Vector3f f_r = intersection.m->eval(obj2LightDir, obj2ViewDir, N);
        float cosTheta = std::max(0.f, dotProduct(obj2LightDir, N));
        float cosTheta_ = std::max(0.f, dotProduct(-obj2LightDir, NN));
        float x_p_2 = dotProduct(obj2Light, obj2Light);
        L_dir = emit * f_r * cosTheta * cosTheta_ /x_p_2 / pdf_light;
    }

    if(get_random_float()<RussianRoulette){
        Vector3f obj2NextObj = intersection.m->sample(obj2ViewDir, N).normalized();
        float pdf = intersection.m->pdf(obj2ViewDir, obj2NextObj, N);
        if(pdf>epsilon){
            Intersection interNext = intersect(Ray(hitPoint, obj2NextObj));
            if(interNext.happened && !interNext.m->hasEmission()){
                Vector3f f_r = intersection.m->eval(obj2NextObj, obj2ViewDir, N);
                float cosTheta = std::max(0.f, dotProduct(obj2NextObj, N));
                L_indir = shade(interNext, -obj2NextObj) * f_r * cosTheta / pdf / RussianRoulette; 
            }
        }
    }
    return L_dir + L_indir;
}

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection intersection = intersect(ray);
    if(!intersection.happened)  return Vector3f{};
    return shade(intersection, -ray.direction);
}