#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes) {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; ++i) {
            Vector2D pos = start + (end - start) * (float)i / (float)(num_nodes - 1);
            Mass* m = new Mass(pos, node_mass, false);
            masses.push_back(m);
        }
        for (int i = 0; i < num_nodes - 1; ++i) {
            Spring* s = new Spring(masses[i], masses[i + 1], k);
            springs.push_back(s);
        }
        // Comment-in this part when you implement the constructor
        for (auto& i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity) {
        for (auto& s : springs) {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            float len = (s->m1->position - s->m2->position).norm();
            s->m1->forces += s->k * (s->m2->position - s->m1->position) * (len - s->rest_length) / len;
            s->m2->forces += s->k * (s->m1->position - s->m2->position) * (len - s->rest_length) / len;
        }

        for (auto& m : masses) {
            if (!m->pinned) {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // m->forces += gravity * m->mass;
                // m->velocity += m->forces / m->mass * delta_t;
                // m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping
                float damping_factor = 0.005;
                m->forces += gravity * m->mass;
                m->forces += -damping_factor * m->velocity;
                m->velocity += m->forces / m->mass * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
        for (auto& s : springs) {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float len = (s->m1->position - s->m2->position).norm();
            s->m1->forces += s->k * (s->m2->position - s->m1->position) * (len - s->rest_length) / len;
            s->m2->forces += s->k * (s->m1->position - s->m2->position) * (len - s->rest_length) / len;
        }

        for (auto& m : masses) {
            if (!m->pinned) {
                m->forces += gravity * m->mass;
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // m->position += (m->position - m->last_position) + m->forces / m->mass * delta_t * delta_t;
                // m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.00005;
                m->position += (1 - damping_factor) * (m->position - m->last_position) + m->forces / m->mass * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
