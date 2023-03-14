#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = dot(r.d, r.d), b = 2 * dot((r.o - this->o), r.d), c = dot((r.o - this->o), (r.o - this->o)) - this->r2;
  double tmp = b * b - 4 * a * c;
  bool result = false;
  if (tmp < 0)
    result = false;
  else {
    double tmp1 = (-b - sqrt(tmp)) / (2 * a);
    double tmp2 = (-b + sqrt(tmp)) / (2 * a);
    if (tmp1 >= r.min_t && tmp2 <= r.max_t) {
      t1 = tmp1;
      t2 = tmp2;
      r.max_t = (t1 > 0) ? t1 : t2;
      result = true;
    }
  }
  return result;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  return test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
  bool res = test(r, t1, t2);
  if (!res)
    return res;

  i->bsdf = this->get_bsdf();
  i->primitive = this;
  i->t = r.max_t;
  Vector3D norm = (r.max_t * r.d + r.o) - this->o;
  norm.normalize();
  i->n = norm;
  return res;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
