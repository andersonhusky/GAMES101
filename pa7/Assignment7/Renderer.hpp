//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);
    void static RenderTh(std::vector<Vector3f> &framebuffer, const Scene &scene, int startH, int endH, int spp);
    void BuildThread(const Scene& scene);

private:
};
