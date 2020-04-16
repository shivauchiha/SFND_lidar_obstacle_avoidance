/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    
	void insert_helper(Node *&node,std::vector<float> data,uint depth,int id)
	{   uint pos = (depth+2)%3;
		if(node == NULL)
		{
			node = new Node(data,id);
		}
		
		else if(data[pos] < node->point[pos])
		{
			insert_helper(node->left,data,depth+1,id);
		}
		else
		{
			insert_helper(node->right,data,depth+1,id);
		}
		
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert_helper(root,point,1,id);


	}

	void search_helper(Node* node,uint depth,std::vector<float> target,float distanceTol,std::vector<int> &ids)
	{
		if(node !=NULL)
		{
		 //checking if the node is within the target region
		 if(node->point[0] <= target[0]+distanceTol && node->point[0] >= target[0]-distanceTol && node->point[1] >= target[1]-distanceTol && node->point[1] <= target[1]+distanceTol && node->point[2] <= target[2]+distanceTol && node->point[2] >= target[2]-distanceTol )
		 {   
			 float dist = sqrt(fabs(pow(node->point[0]-target[0],2)+pow(node->point[1]-target[1],2)+pow(node->point[2]-target[2],2)));
			 if (dist <= distanceTol)
			 {
				 ids.push_back(node->id);
			 }
			 
		 }

		 //node traversal
		 int cd = (depth+2)%3;
		 if(node->point[cd] > target[cd]-distanceTol)
		 {
           search_helper(node->left,depth+1,target,distanceTol,ids);
		 }
		 if(node->point[cd] < target[cd]+distanceTol)
		 {
           search_helper(node->right,depth+1,target,distanceTol,ids);
		 }

		}
		//else
		//{
			//std::cout<<"The tree is empty or search has ended end of tree"<<std::endl;
		//}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)

	{
		std::vector<int> ids;
		search_helper(root,1,target,distanceTol,ids);
		return ids;
	}
	

};




