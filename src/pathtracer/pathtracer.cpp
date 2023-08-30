#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
    
    for (int i = 0; i < num_samples; i++) {
        Vector3D w_in = hemisphereSampler->get_sample();
        Vector3D w_inWorld = o2w * w_in;
        Ray r = Ray(hit_p + (EPS_F * w_inWorld), w_inWorld);
        r.min_t = EPS_F;
        Intersection intersection;
        
        if (bvh->intersect(r, &intersection)) {
            L_out += isect.bsdf->f(w_out, w_in) * zero_bounce_radiance(r, isect) * cos_theta(w_in);
        }
    }
    return (L_out * 2.0 * PI)/(double)num_samples;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
    
  int num_samples = scene->lights.size() * ns_area_light;
    
  for (auto light = scene -> lights.begin(); light != scene -> lights.end(); light++) {
      int sample = 1;
      Vector3D L_out_total;
      while (sample <= ns_area_light && !((*light) -> is_delta_light())) {
          Vector3D wi_d;
          double distToLight;
          double pdf;
          Vector3D L = (*light) -> sample_L(hit_p, &wi_d, &distToLight, &pdf);

          Ray ray = Ray(hit_p, wi_d, 1);
          ray.min_t = EPS_F;
          ray.max_t = (double)distToLight - ray.min_t;

          Intersection inter;
          if (dot(wi_d, isect.n) > 0 && !(bvh -> intersect(ray, &inter))) {
              L_out_total += L * isect.bsdf -> f(w_out, wi_d) * dot(wi_d, isect.n) / pdf;
          }
          sample += 1;
      }

    L_out += L_out_total / sample;
  }

//    L_out = L_out_total/sample;
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
    return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

    if (direct_hemisphere_sample) {
        return estimate_direct_lighting_hemisphere(r, isect);
    } else {
        return estimate_direct_lighting_importance(r, isect);
    }

}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);
    // TODO: Part 4, Task 2
    // Returns the one bounce radiance + radiance from extra bounces at this point.
    // Should be called recursively to simulate extra bounces.
    
    Vector3D L_out = one_bounce_radiance(r, isect);

    if (max_ray_depth <= 1) {
        return L_out;
    }
    
    if (coin_flip(0.65)) {
          Vector3D w_in;
          double pdf;
          Vector3D L = isect.bsdf -> sample_f(w_out, &w_in, &pdf);
          Ray r_next = Ray(hit_p + (EPS_F * o2w * w_in), o2w * w_in);

          r_next.depth = r.depth - 1;
          r_next.min_t = EPS_F;
          Intersection inter;

          if (bvh -> intersect(r_next, &inter)) {
              //recursion
              L_out += at_least_one_bounce_radiance(r_next, inter) * L * cos_theta(w_in) / pdf / 0.65;
          }
    }
    return L_out;
    
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  // TODO (Part 3): Return the direct illumination.
    
//    L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
    
//    L_out = zero_bounce_radiance(r, isect) + estimate_direct_lighting_importance(r, isect);
    
//    L_out = zero_bounce_radiance(r, isect) + estimate_direct_lighting_hemisphere(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
    L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  return L_out;
}


void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.
    float s1 = 0;
    float s2 = 0;
    int n = 0;
    Vector3D radiancetotal(0, 0, 0);
    Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
    for (int i = 0, batchsize = 1; i < ns_aa; i++, batchsize++) {
        n += 1;
        
        Vector2D sample = gridSampler->get_sample();
        Ray ray = camera->generate_ray((sample.x + x)/sampleBuffer.w,
                                       (sample.y + y)/sampleBuffer.h);
        
        float xk = est_radiance_global_illumination(ray).illum();
        s1 += xk;
        s2 += std::pow(xk, 2);
        Vector3D v = est_radiance_global_illumination(ray);
        radiancetotal += v;
        
        if (batchsize == samplesPerBatch) { //tip 3
            batchsize = 0;
            float variance = (1 / ((double)n - 1)) * (s2 - (s1 * s1) / (double)n);
            float convergence = 1.96 * std::sqrt(variance) / std::sqrt((double)n);
            if (convergence <= maxTolerance * s1 / (float)n) {
                //cout << "SDFDSF";
                break;
            }
        }
        

    }
    sampleBuffer.update_pixel(radiancetotal / n, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = n; //idk?

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
