#include "fdg.h"

const float min_force = 0.0f;
const int max_iterations = 100;
const float attraction = 750.0f;//attraction force multiplier, between 0 and much
const float repulsion = 1000.0f;//repulsion force multiplier, between 0 and much
const int length = 200; //spring length in units aka the distance an edge should be long
float cooldown = 0.98f;

/// <summary>
/// kind of bad practice because of side effects, but: 
/// this copies the arrays of ints for the x and y coordinates given the count. 
/// then some force directed graphing algorithms are performed, 
/// and written back to the addresses gioven by the pointers. 
/// </summary>
/// <param name="x">x coord of the nodes</param>
/// <param name="y">y coord of the nodes</param>
/// <param name="mass">mass of the nodes</param>
/// <param name="count">max count of all nodes</param>
extern void __stdcall do_graph_physics(int32_t* x, int32_t* y, int32_t* mass, int32_t count, int32_t* node_1, int32_t* node_2, int32_t node_connection_count) {
	//we need some nodes or else things break
	if (node_connection_count > 0) {
		float current_max_force = min_force + 0.1f;

		//copy node ids in an array so we can apply force and not disturb the rest
		//save all forces here
		float* x_node_forces = static_cast<float*>(calloc(count, sizeof(float)));
		float* y_node_forces = static_cast<float*>(calloc(count, sizeof(float)));

		//times to perform calculation, result gets better over time
		int iteration = 0;
		while (iteration < max_iterations && current_max_force > min_force) {
			//calculate new, smaller cooldown so the nodes will move less and less
			cooldown *= cooldown;

			for (int current = 0; current < count; current++) {
				for (int other = 0; other < count; other++) {
					//difference as a vector
					int x_diff = x[other] - x[current];
					int y_diff = y[other] - y[current];
					//if not the same node and not same position
					if (current != other && x_diff && y_diff) {
						//absolute length of difference/distance (sqrt of distance)
						float distance = sqrtf(powf(static_cast<float>(x_diff), 2) + powf(static_cast<float>(y_diff), 2));
						//add force like this: f_rep = c_rep / (distance^2) * vec(p_u->p_v)
						//Vector2 repulsionForce = repulsion / (distance * distance) * (difference / distance) / node.Mass;
						float x_rep = repulsion / powf(distance, 2) * (x_diff / distance) / mass[current];
						float y_rep = repulsion / powf(distance, 2) * (y_diff / distance) / mass[current];
						//go through all edges

						for (int edge = 0; edge < node_connection_count; edge++) {
							//if we have a connection from us to the node we are locking at right now, aka child node
							if (node_1[edge] == current && node_2[edge] == other) {
								//so we can now do the attraction force
								//formula: c_spring * log(distance / length) * vec(p_u->p_v) - f_rep
								//NodeForces[node.Guid] += (attraction * (float)Math.Log(distance / length) * (difference / distance)) - repulsionForce;
								x_node_forces[current] += (attraction * logf(distance / length) * (x_diff / distance)) - x_rep;
								y_node_forces[current] += (attraction * logf(distance / length) * (y_diff / distance)) - y_rep;
								break;
							}
							else {
								x_node_forces[current] += x_rep;
								y_node_forces[current] += y_rep;
								break;
							}
						}
						//add new maximum force or keep it as is if ours is smaller
						float force_diag = sqrtf((x_node_forces[current] * x_node_forces[current]) + (y_node_forces[current] * y_node_forces[current]));
						current_max_force = current_max_force > force_diag ? current_max_force : force_diag;
					}
				}
			}

			//apply forces
			for (int i = 0; i < count; i++) {
				x[i] += static_cast<int>(cooldown * x_node_forces[i]);
				y[i] += static_cast<int>(cooldown * y_node_forces[i]);
			}

			iteration++;
		}

		//free our forces after use
		free(x_node_forces);
		free(y_node_forces);
	}
}
