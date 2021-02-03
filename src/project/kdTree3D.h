/* \author Aaron Brown */
// Quiz on implementing kd tree
#include <stdint.h>

#include "../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree3D
{
	Node* root;

	KdTree3D()
		: root(NULL)
	{}
	void insertHelper(Node** node, uint8_t depth, std::vector<float> point, int id)
	{
		// Tree is empty
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			// Calculate current dim 
			// Depth cases-> (0,1,2)=(x,y,z)
			uint8_t curentDepth = depth % 3;

			if (point[curentDepth] < ((*node)->point[curentDepth]))
			{
				insertHelper(&((*node)->left), depth + 1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth + 1, point, id);
			}

		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
				(node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))  &&
				(node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)))
			{
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
									  (node->point[1] - target[1]) * (node->point[1] - target[1]) +
									  (node->point[2] - target[2]) * (node->point[2] - target[2]));
				
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}
			// Check accross boundary
			uint8_t curentDepth = depth % 3;
			
			if ((target[curentDepth] - distanceTol) < node->point[curentDepth])
			{
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}
			if ((target[curentDepth] + distanceTol) > node->point[curentDepth])
			{
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}


};




