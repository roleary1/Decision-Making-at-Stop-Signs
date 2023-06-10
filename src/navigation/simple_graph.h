#include <map>
#include "eigen3/Eigen/Dense"

using namespace std;

namespace navigation {
	struct Node {
		Eigen::Vector2f pos;
		// Start with 12 o'clock, counted clockwise
		int edges[8];
		Node() : pos(0, 0), edges{-1, -1, -1, -1, -1, -1, -1, -1} {};
	};

	class Graph {
		public:
		// Configuration
		float actual_dist;
		float start_x;
		float start_y;
		float end_x;
		float end_y;

		// Useful to us
		int column_height;

		map <int, Node> nodes;

		Graph() {};

		Graph(float dist, float start_x, float start_y, float end_x, float end_y) 
			: actual_dist(dist), start_x(start_x), start_y(start_y), end_x(end_x), end_y(end_y) {
				column_height = (int)((end_y - start_y)/actual_dist);
		};
		
		// Get Node Number from position
		// Node 0 is at (start_x, start_y), Node 1 is at (start_x, start_y+actual_dist)
		// Node column_height is at (start_x+actual_dist, start_y)
		int getNodeNumber(Eigen::Vector2f pos) {
			// Round position to nearest node
			float round_value = 1/actual_dist;
			float rounded_x = round((pos[0])*round_value) / round_value;
			float rounded_y = round((pos[1])*round_value) / round_value;

			// Get node position
			int x_offset = (int)((rounded_x - start_x) / actual_dist);
			int y_offset = (int)((rounded_y - start_y) / actual_dist);

			return x_offset * column_height + y_offset;
		};

		// Accept direction from 0-7, return adjustment to node value
		int getDirectionAdjust(int direction) {
			switch(direction) {
				case 0:
					return 1;
				case 1:
					return column_height+1;
				case 2:
					return column_height;
				case 3:
					return column_height-1;
				case 4:
					return -1;
				case 5:
					return -column_height-1;
				case 6:
					return -column_height;
				case 7:
					return -column_height+1;
				default:
					return 0;
			}
		};

		float getCost(int direction) {
			if(direction % 2 == 0) {
				return actual_dist;
			} else {
				return actual_dist * sqrt(2);
			}
		};
	};
}