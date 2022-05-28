#include "fdg.h"

const int max_iterations = 100;
const float attraction = 750.0f;//attraction force multiplier, between 0 and much
const float repulsion = 1000.0f;//repulsion force multiplier, between 0 and much
const int length = 200; //spring length in units aka the distance an edge should be long
float cooldown = 0.97f;

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

		//copy node ids in an array so we can apply force and not disturb the rest
		//save all forces here
		float* x_node_forces = static_cast<float*>(calloc(2 * static_cast<size_t>(count), sizeof(float)));
		float* y_node_forces = x_node_forces + count;

		//check for null 
		if (x_node_forces) {

			//preallocate all variables so we dont have to create new ones all the time
			int edge = 0;
			float rep = 0, distance = 0, x_rep = 0, y_rep = 0, x_diff = 0, y_diff = 0;

			//times to perform calculation, result gets better over time
			int iteration = 0;
			while (iteration < max_iterations) {
				//calculate new, smaller cooldown so the nodes will move less and less
				cooldown *= cooldown;

				for (int current = 0; current < count; current++) {
					for (int other = 0; other < count; other++) {
						//difference as a vector
						x_diff = x[other] - x[current];
						y_diff = y[other] - y[current];
						//if not the same node and not same position
						if (current != other && x_diff && y_diff) {
							//absolute length of difference/distance (sqrt of distance)
							distance = sqrtf(powf(x_diff, 2) + powf(y_diff, 2));

							//add force like this: f_rep = c_rep / (distance^2) * vec(p_u->p_v)
							//Vector2 repulsionForce = repulsion / (distance * distance) * (difference / distance) / node.Mass;
							x_rep = (repulsion * x_diff) / (mass[current] * powf(distance, 2) * distance);
							y_rep = (repulsion * y_diff) / (mass[current] * powf(distance, 2) * distance);

							//check next edge we have in our list
							//if we have a connection from us to the node we are locking at right now, aka child node
							if (node_1[edge] == current && node_2[edge] == other) {
								//so we can now do the attraction force
								//formula: c_spring * log(distance / length) * vec(p_u->p_v) - f_rep
								//NodeForces[node.Guid] += (attraction * (float)Math.Log(distance / length) * (difference / distance)) - repulsionForce;
								x_node_forces[current] += (attraction * logf(distance / length) * (x_diff / distance)) - x_rep;
								y_node_forces[current] += (attraction * logf(distance / length) * (y_diff / distance)) - y_rep;

								//increase iterator to next edge
								edge++;
							}
							else {
								x_node_forces[current] += x_rep;
								y_node_forces[current] += y_rep;
							}
						}
					}
				}

				//apply forces
				for (int i = 0; i < count; i++) {
					x[i] += cooldown * x_node_forces[i];
					y[i] += cooldown * y_node_forces[i];
				}

				iteration++;
			}
			//free our forces after use
			free(x_node_forces);
		}
	}
}
