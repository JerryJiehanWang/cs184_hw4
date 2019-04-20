#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double unit_width = width / (double) num_width_points;
  double unit_height = height / (double) num_height_points;
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      if (orientation == HORIZONTAL) {
        point_masses.emplace_back(Vector3D((double) x * unit_width, 1, (double) y * unit_height), false);
      } else if (orientation == VERTICAL) {
        double z =  - 0.001 + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(0.002)));
        point_masses.emplace_back(Vector3D((double) x * unit_width, (double) y * unit_height, z), false);
      }
    }
  }

  for (vector<int> pin: pinned) {
    int x = pin[0];
    int y = pin[1];
    point_masses[x + y * num_width_points].pinned = true;
  }

  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      //structural force
      PointMass *point = &point_masses[x + y * num_width_points];
      PointMass *above = point - num_width_points;
      PointMass *left = point - 1;
      PointMass *upperLeft = point - num_width_points - 1;
      PointMass *upperRight = point - num_width_points + 1;
      PointMass *two_left = point - 2;
      PointMass *two_above = point - 2 * num_width_points;

      if (x != 0) {
        springs.emplace_back(left, point, STRUCTURAL);
      }

      if (y != 0) {
        springs.emplace_back(above, point, STRUCTURAL);
      }

      if (x != 0 && y != 0) {
        springs.emplace_back(upperLeft, point, SHEARING);
      }

      if (x != num_width_points - 1 && y != 0) {
        springs.emplace_back(upperRight, point, SHEARING);
      }

      if (x > 1) {
        springs.emplace_back(two_left, point, BENDING);
      }

      if (y > 1) {
        springs.emplace_back(two_above, point, BENDING);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  //(Part 2): Compute total force acting on each point mass.

  //external forces
  Vector3D totalF = Vector3D(0,0,0);
  for (Vector3D acceleration: external_accelerations) {
    totalF +=  mass * acceleration;
  }

  for (int i = 0; i < point_masses.size(); i++) {
    point_masses[i].forces = totalF;
  }

  //Spring correction force
  double ks = cp -> ks;
  for (int i = 0; i < springs.size(); i++) {
    double l = springs[i].rest_length;
    double distance = (springs[i].pm_a -> position - springs[i].pm_b -> position).norm();
    Vector3D direction = (springs[i].pm_a -> position - springs[i].pm_b -> position).unit();

    double F =  ks * (distance - l);
    if ((springs[i].spring_type == STRUCTURAL && cp -> enable_structural_constraints) ||
    (springs[i].spring_type == SHEARING && cp -> enable_shearing_constraints)) {
      springs[i].pm_a -> forces -= F * direction;
      springs[i].pm_b -> forces += F * direction;
    } else if (springs[i].spring_type == BENDING && cp -> enable_bending_constraints){
      springs[i].pm_a -> forces -= 0.2 * F * direction;
      springs[i].pm_b -> forces += 0.2 * F * direction;
    }
  }

  //(Part 2): Use Verlet integration nto compute new point mass positions
  for (int i = 0; i < point_masses.size(); i++) {
    if (!point_masses[i].pinned) {
      Vector3D x_t = point_masses[i].position;
      Vector3D x_t_1 = point_masses[i].last_position;
      Vector3D a = point_masses[i].forces / mass;
      double d = cp -> damping / 100.0;
      point_masses[i].position = x_t + ((double) 1.0 - d) * (x_t - x_t_1) + a * pow(delta_t, 2);
      point_masses[i].last_position = x_t;
    }
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();

  for (int i = 0; i < point_masses.size(); i++) {
    self_collide(point_masses[i], simulation_steps);
  }

  //(Part 3): Handle collisions with other primitives.
  for (int i = 0; i < point_masses.size(); i++) {
    for (CollisionObject *c: *collision_objects) {
      c -> collide(point_masses[i]);
    }
  }

  //(Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

  for (int i = 0; i < springs.size(); i++) {

    double distance = (springs[i].pm_a -> position - springs[i].pm_b -> position).norm();
    Vector3D direction = (springs[i].pm_a -> position - springs[i].pm_b -> position).unit();

    if (distance > 1.1 * springs[i].rest_length) {
      double difference = distance - 1.1 * springs[i].rest_length;

      if (!springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
        springs[i].pm_a->position -= (difference / 2.0) * direction;
        springs[i].pm_b->position += (difference / 2.0) * direction;
      } else if (!springs[i].pm_a->pinned && springs[i].pm_b->pinned) {
        springs[i].pm_a->position -= difference * direction;
      } else if (springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
        springs[i].pm_b->position += difference * direction;
      }
    }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < point_masses.size(); i++) {
    float index = hash_position(point_masses[i].position);

    if (map[index] == NULL) {
      map[index] = new vector<PointMass *>();
    }

    map[index] -> push_back(&point_masses[i]);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

  float index = hash_position(pm.position);
  vector<PointMass *> *collide_points = map[index];

  Vector3D correction = Vector3D(0, 0, 0);
  int count = 0;

  for (int i = 0; i < collide_points -> size(); i++) {
    Vector3D collide_pos = (*collide_points)[i] -> position;
    double distance = (pm.position - collide_pos).norm();
    if (distance <= 2 * thickness && &pm != (*collide_points)[i]) {
      count += 1;
      double diff_distance = 2 * thickness - distance;
      Vector3D direction = (pm.position - collide_pos).unit();
      correction += direction * diff_distance;
    }
  }

  if (count != 0) {
    pm.position += correction / (double) count / simulation_steps;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3 * width / (double) num_width_points;
  double h = 3 * height / (double) num_height_points;
  double t = max(w, h);

  Vector3D new_pos = Vector3D(floor(pos[0] / w), floor(pos[1] / h), floor(pos[2] / t));

  return new_pos[0] * 17.0 + new_pos[1] * 7 + new_pos[2] * 3;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
