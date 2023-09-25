//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const {
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const {
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
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

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject) {
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
Vector3f Scene::castRay(const Ray& ray, int depth) const {
    // TODO Implement Path Tracing Algorithm here
    /*
    伪代码：
    shade(p, wo)
        sampleLight(inter, pdf_light)
        Get x, ws, NN, emit from inter
        Shoot a ray from p to x
        If the ray is not blocked in the middle
            L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p| ^ 2 / pdf_light

        L_indir = 0.0
        Test Russian Roulette with probability RussianRoulette
        wi = sample(wo, N)
        Trace a ray r(p, wi)
        If ray r hit a non-emitting object at q
            L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette

        return L_dir + L_indir
    */
    Intersection inter = intersect(ray);
    // 光线没有命中物体
    if (!inter.happened) {
        return Vector3f(0.0, 0.0, 0.0);
    }
    // 光线命中光源，返回光源颜色
    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }
    // 光线命中物体
    Vector3f L_dir = 0.0, L_indir = 0.0;
    float pdf_light;
    Vector3f p = inter.coords;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = ray.direction;
    Intersection inter_light;
    sampleLight(inter_light, pdf_light);
    Vector3f x = inter_light.coords;
    Vector3f ws = (x - p).normalized();
    Vector3f NN = inter_light.normal.normalized();
    Vector3f emit = inter_light.emit;
    // 如果没有物体阻挡
    // if (inter_light.distance - intersect(Ray(p, ws)).distance < EPSILON) {
    // sample() 函数中没有计算 inter_light.distance
    if ((x - p).norm() - intersect(Ray(p, ws)).distance < EPSILON) {
        L_dir = emit * inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / std::pow((x - p).norm(), 2) / pdf_light;
    }
    // RussianRoulette 概率
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = inter.m->sample(wo, N).normalized();
        Intersection inter_obj = intersect(Ray(p, wi));
        // 命中不发光物体
        if (inter_obj.happened && !inter_obj.m->hasEmission()) {
            float pdf_obj = inter.m->pdf(wo, wi, N);
            L_indir = castRay(Ray(p, wi), depth + 1) * inter.m->eval(wo, wi, N) * dotProduct(wi, N) / pdf_obj / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}