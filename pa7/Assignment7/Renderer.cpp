//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
int Total = 0;
std::mutex framebuffer_lock;
std::mutex Total_loc;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    // 采样数量
    int spp = 128;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + get_random_float()) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            // 执行spp次光线追踪
            Ray ray = Ray(eye_pos, dir);
            Intersection intersection = scene.intersect(ray);
            if(intersection.happened){
                for (int k = 0; k < spp; k++){
                    framebuffer[m] += scene.shade(intersection, -ray.direction) / spp;  
                }
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

void Renderer::RenderTh(std::vector<Vector3f> &framebuffer, const Scene &scene, int startH, int endH, int spp)
{
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;
    int m_start = startH*scene.width;

    for (uint32_t j = startH; j < endH; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            // 执行spp次光线追踪
            Ray ray = Ray(eye_pos, dir);
            Intersection intersection = scene.intersect(ray);
            if(intersection.happened){
                for (int k = 0; k < spp; k++){
                    Vector3f  pixel = scene.shade(intersection, -ray.direction) / spp;
                    // framebuffer_lock.lock();
                    framebuffer[m_start+m] += pixel;
                    // framebuffer_lock.unlock();
                }
            }
            m++;
        }
        Total_loc.lock();
        Total++;
        UpdateProgress(Total / (float)scene.height);
        Total_loc.unlock();
    }
}

void Renderer::BuildThread(const Scene& scene){
    int spp=128;
    std::cout << "SPP: " << spp << "\n";

    int NUM_THREADS = std::thread::hardware_concurrency();
    int onePart = scene.height / NUM_THREADS + 1;
    std::vector<std::thread> RenderThreads;
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    for(int i=0; i<NUM_THREADS; i++)
    {
        int startH = onePart * i;
        int endH = std::min(scene.height, startH+onePart);
        std::cout << "Create NO." << i+1 << " thread, process line " << startH << " to line " << endH << std::endl;
        RenderThreads.push_back(std::thread(Renderer::RenderTh, std::ref(framebuffer), std::ref(scene), startH, endH, spp));
    }
    for(auto itr=RenderThreads.begin(), end=RenderThreads.end(); itr!=end; itr++)
    {
        itr->join();
    }

    UpdateProgress(1.f);
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}