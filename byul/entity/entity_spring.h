#ifndef ENTITY_SPRING_H
#define ENTITY_SPRING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"
#include "trajectory.h"

// ---------------------------------------------------------
// Spring Force (Hooke's Law)
// ---------------------------------------------------------
/**
 * @brief Calculate the spring force between two entities.
 *
 * The spring model is based on Hooke's Law and damping force:
 *
 * F = -k (d - L0) * d_hat - c (v_rel * d_hat) * d_hat
 *
 * - d : Distance between two entities
 * - L0 : Natural length of the spring (target distance)
 * - k : Spring stiffness coefficient (higher = stronger force)
 * - c : Damping coefficient (>= 0)
 * - d_hat : Unit vector between the two entities
 *
 * @param[out] out   Calculated force vector (ignored if NULL)
 * @param[in]  a     Reference entity (receiving the force)
 * @param[in]  b     Other entity
 * @param[in]  k     Spring stiffness coefficient (k > 0)
 * @param[in]  c     Damping coefficient (>= 0)
 * @param[in]  L0    Target distance (m)
 *
 * @note
 * - d < L0 -> Repulsion (push)
 * - d > L0 -> Attraction (pull)
 * - d = L0 -> No force
 */
BYUL_API void spring_force(vec3_t* out,
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    float k,
    float c,
    float L0);

/**
 * @brief Calculate the total spring force acting on one entity from multiple others.
 *
 * Calls spring_force() for all other entities and sums the results.
 *
 * @param[out] out   Total force vector (ignored if NULL)
 * @param[in]  self  Reference entity
 * @param[in]  others Array of other entities
 * @param[in]  count Number of entities in others
 * @param[in]  k     Spring stiffness coefficient
 * @param[in]  c     Damping coefficient
 * @param[in]  L0    Target distance
 */
BYUL_API void spring_force_total(vec3_t* out,
    const entity_dynamic_t* self,
    const entity_dynamic_t* others,
    int count,
    float k,
    float c,
    float L0);

/**
 * @brief Perform a full spring-based distance maintenance simulation.
 *
 * Each entity is assumed to be connected to all other entities by springs.
 * Hooke's Law and damping are applied to maintain the natural length L0.
 *
 * @param[out] traj   Pointer to trajectory record (can be NULL)
 * @param[in,out] e   Array of entities
 * @param[in]  count  Number of entities
 * @param[in]  dt     Time step (seconds)
 * @param[in]  k      Spring stiffness coefficient
 * @param[in]  c      Damping coefficient
 * @param[in]  L0     Target distance (m)
 * @param[in]  steps  Number of simulation iterations
 *
 * @note
 * Uses Semi-Implicit Euler Integrator:
 *
 * v(t+1) = v(t) + (F/m) * dt
 * x(t+1) = x(t) + v(t+1) * dt
 */
BYUL_API void spring_simulate(trajectory_t* traj,
    entity_dynamic_t* e, int count,
    float dt, float k, float c, float L0,
    int steps);

// ---------------------------------------------------------
// Pairwise Spring Simulation
// ---------------------------------------------------------
/**
 * @brief Perform a pairwise spring simulation between all entity pairs.
 *
 * For every entity pair (i, j), spring force is computed and positions and velocities are updated.
 *
 * @param[out] traj   Pointer to trajectory record (can be NULL)
 * @param[in,out] e   Array of entities
 * @param[in]  count  Number of entities
 * @param[in]  dt     Time step (seconds)
 * @param[in]  k      Spring stiffness coefficient
 * @param[in]  c      Damping coefficient
 * @param[in]  L0     Natural length
 * @param[in]  steps  Number of simulation steps
 */
BYUL_API void spring_simulate_pairwise(trajectory_t* traj,
                              entity_dynamic_t* e, int count,
                              float dt, float k, float c, float L0,
                              int steps);

// ---------------------------------------------------------
// Spring Network (Graph-based)
// ---------------------------------------------------------
/**
 * @struct spring_link_t
 * @brief Structure representing a spring connection between two entities.
 *
 * - i, j : Indices of connected entities
 * - k : Spring stiffness coefficient
 * - c : Damping coefficient
 * - L0: Natural length
 */
typedef struct s_spring_link {
    int i, j;
    float k;
    float c;
    float L0;
} spring_link_t;

/**
 * @brief Perform a network (graph-based) spring simulation.
 *
 * Uses an entity array and spring link information (links)
 * to compute spring forces across the network and update positions and velocities.
 *
 * @param[out] traj       Pointer to trajectory record (NULL = no logging)
 * @param[in,out] e       Array of entities
 * @param[in]  count      Number of entities
 * @param[in]  links      Array of spring links
 * @param[in]  link_count Number of links
 * @param[in]  dt         Time step (seconds)
 * @param[in]  steps      Number of simulation steps
 *
 * @note
 * - spring_simulate_pairwise() computes all pairs,
 *   spring_simulate_network() only computes for specified links[].
 */
BYUL_API void spring_simulate_network(
    trajectory_t* traj,
    entity_dynamic_t* e, int count,
    const spring_link_t* links, int link_count,
    float dt, int steps);

/**
 * @brief Perform a repulsion network simulation using Velocity Verlet integration.
 *
 * This function simulates a repulsion network based on virtual springs between entities.
 * Each link (spring_link_t) defines a minimum safe distance (L0)
 * and applies repulsion and damping forces if entities get closer than this distance.
 * The simulation updates positions and velocities using Velocity Verlet integration.
 *
 * Main Features:
 * - Each entity is modeled as a particle with mass (props.mass), velocity, and position (xf.pos).
 * - The links array defines spring constant k, damping coefficient c, and target distance L0 for each pair.
 * - On each step, forces from all links are accumulated, then positions and velocities are updated.
 * - If trajectory_t is provided, the state (position/velocity) of each entity is recorded at each step.
 *
 * @param[out] traj         Pointer to trajectory record (NULL = no logging)
 * @param[in,out] e         Array of entities (entity_dynamic_t[count])
 * @param[in] count         Number of entities (must be >= 2)
 * @param[in] links         Array of repulsion links (spring_link_t[link_count])
 * @param[in] link_count    Number of links
 * @param[in] dt            Time step (seconds)
 * @param[in] steps         Number of simulation steps
 *
 * @note
 * - If traj is NULL, only the final state is updated.
 * - Velocity Verlet is more stable than Euler due to its half-step velocity update.
 * - All entities must have props.mass > 0 for correct force-to-acceleration calculations.
 *
 * @warning
 * - If count <= 1 or link_count <= 0, no simulation is performed.
 * - Large dt may cause instability; 0.01 to 0.05 seconds is recommended.
 *
 * @see spring_link_t
 * @see entity_dynamic_t
 * @see trajectory_t
 */
BYUL_API void repulsion_simulate_network(trajectory_t* traj,
                                entity_dynamic_t* e, int count,
                                const spring_link_t* links, int link_count,
                                float dt, int steps);

/**
 * @brief Perform simulation using external push forces to move entities smoothly.
 *
 * - Each entity has mass (props.mass), position (xf.pos), and velocity.
 * - The push_forces array defines the external force applied to each entity.
 * - Instead of springs, a constant force is applied for smooth pushing like human interaction.
 * - If trajectory_t is provided, the state is recorded at each step.
 *
 * @param[out] traj        Pointer to trajectory record (NULL = no logging)
 * @param[in,out] e        Array of entities (size = count)
 * @param[in] count        Number of entities
 * @param[in] push_forces  Array of external forces (size = count)
 * @param[in] dt           Time step (seconds)
 * @param[in] steps        Number of simulation steps
 */
BYUL_API void push_simulate_network(trajectory_t* traj,
                                    entity_dynamic_t* e, int count,
                                    const vec3_t* push_forces,
                                    float dt, int steps);                                

#ifdef __cplusplus
}
#endif

#endif // ENTITY_SPRING_H
