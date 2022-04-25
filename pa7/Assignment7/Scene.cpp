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
Vector3f Scene::castRay(const Ray &ray, int depth, Intersection& intersection) const
{
    // TO DO Implement Path Tracing Algorithm here
    //1、判断观察方向是否看到有颜色的物体
    // 2、在观察点随机采样一个入射方向，返回入射方向的光度贡献

    Material *m = intersection.m;
    Object * hitObject = intersection.obj;
    Vector3f L_dir=Vector3f(0.f), L_indir=Vector3f(0.f);
    // Vector3f hitColor = this->backgroundColor;

    float uv;
    uint32_t index = 0;

    if(intersection.happened){
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal;
        Vector2f st;
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);

        // -------------light part-------------------
        Intersection inter;
        float pdf_light;
        sampleLight(inter, pdf_light);
        // 随机的光源面上的点、法向量、光度
        Vector3f hitLightPoint = inter.coords;
        Vector3f NN = inter.normal;
        Vector3f emit = inter.emit;

        Vector3f ws = normalize(hitLightPoint-hitPoint);
        // Ray rayTest = Ray(hitPoint, ws);
        // Intersection interTest = Scene::intersect(rayTest);

        Vector3f f_r = m->eval(ray.direction, ws, N);
        float cosTheta = dotProduct(ws, N);
        float cosTheta_ = dotProduct(ws, NN);
        cosTheta = cosTheta<0? -cosTheta: cosTheta;
        cosTheta_ = cosTheta_<0? -cosTheta_: cosTheta_;
        float x_ = (hitLightPoint-hitPoint).norm();
        std::cout << "f_r: " << f_r << ", cosTheta: " <<cosTheta << ", cosTheta_: " <<cosTheta_ << std::endl;
        L_dir = emit * f_r * cosTheta * cosTheta_ / x_ / x_ / pdf_light;
        std::cout << L_dir << std::endl;

        // if(interTest.happened){
        //     Vector3f hitMid = interTest.coords;
        //     float distance = (hitLightPoint-hitMid).norm();
            // if(distance <= 1e-6){
                // Vector3f f_r = m->eval(ray.direction, ws, N);
                // float cosTheta = dotProduct(ws, N);
                // float cosTheta_ = dotProduct(ws, NN);
                // float x_ = (hitLightPoint-hitPoint).norm();
                // L_dir = emit * f_r * cosTheta * cosTheta_ / x_ / x_ / pdf_light;
            // }
        // }

        // -------------obj part---------------------
        if(get_random_float()<RussianRoulette){
            // Vector3f wi = normalize(m->sample(ray.direction, N));
            // Ray rayNext = Ray(hitPoint, wi);
            // Intersection interNext = Scene::intersect(rayNext);
            // if(interNext.happened){
            //     Material* mnext = interNext.m;
            //     if(!mnext->hasEmission()){
            //         float pdf_wi = m->pdf(wi, ray.direction, N);
            //         Vector3f f_r = m->eval(wi, ray.direction, N);
            //         float cosTheta = dotProduct(rayNext.direction, N);
            //         L_indir = castRay(rayNext, depth, interNext) * f_r * cosTheta / pdf_wi / RussianRoulette;
            //     }
            // }
        }
    }
    return L_dir + L_indir;
}