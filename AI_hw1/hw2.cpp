#include<iostream>
#include<vector>
#include<queue>
#include<stack>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<list>
#include<math.h>


using namespace std;

#define DEBUG1 0

#define NELEMS(array) (sizeof(array)/sizeof(array[0]))
#define MAXGRIDSIZE 30  //implies to max width or max height
#define MINGRIDSIZE 2   //implies to min width or min height 
#define INVALID -19
// we can have combinations of a 2x2 grid to 30x30 


//global variables
int OPTIONBFS = 0;
int OPTIONDFS = 0;
int OPTIONASTAR = 0;
int OPTIONUCS = 0;

int ALTH = 0; //runs self designed heuristics on Astar
int ALTC = 0; //runs bottom favouring cost function meant only for A* or UCS
int STATS = 0;  //shows extra info




struct WNode
{
	int x;
	int y;        
	char type;           // can be S/Start or E/Goal or N/Wire 
	bool visited;        // 0 is not visited : 1 is visited
	int parentindex;     // will hold the index of the parent node
	double cost;         // will hold cost of node to start node using constant function or bottom favouring
	double heuristic;    // will hold the cost of node to goal node using heuristic
};
typedef struct WNode Node;






//global variables
int gridwidth;
int gridheight;
int totalelemsinfile; //contains total numbers in the file
int startpointx;
int startpointy;
int endpointx;
int endpointy;

//global Nodes and 
Node GridNode[MAXGRIDSIZE*MAXGRIDSIZE];		//Node matrix that has all node information 
Node startNode;				 	//Global variable to hold root node
Node goalNode;  				//Global variable to hold goal node
int GridMatrix[MAXGRIDSIZE][MAXGRIDSIZE];       //matrix to index GRID NODE
int testmatrix[MAXGRIDSIZE*MAXGRIDSIZE*2];      //reads data from file


//Function definitions
int Readmatrixtextfile(char *filename);
void displayNodetype(Node N);
void displayXY(Node N);
void displayfile();
void initializeGridMatrix();
void initializeGridNodes();
void displayGrid();

int getstartNodeindex();
int getgoalNodeindex();
void displayvisited();
void displaytype();

int getindex(int,int);

double ucost_constant(Node parent, Node neighbor);  //cost functions
double ucost_botfavor(Node parent, Node neighbor);  //cost functions
double computeheuristic_manhattan(Node CurrentNode, Node GoalNode); //returns heuristic cost  from current to goal by manhattan function

double selfdefinedheuristics(Node CurrentNode, Node GoalNode);      //returns heuristic cost from Current to Goal node by self designed function

bool operator > (const Node& GridNode1, const Node& GridNode2);


//functions dfs bfs ucs astar
void bfs();
void dfs();
void ucs(); //constant gn
void ucs_bottomfavouring();
void astarmanhattan();  // this uses constant cost gn function and manhattan heuristics
void astarmanhattan_bottomfavor(); //this uses bottom favouring gn function and manhattan heuristics
void astar_selfdefinedheuristics_constgn();
void astar_selfdefinedheuristics_bottomfavor();



typedef priority_queue <Node, vector<Node>, greater<Node> > Priority_Queue;


int main(int argc, char * argv[])
{

/*
./CS561.exe -BFS grid1.txt
./CS561.exe -DFS -stats grid1.txt
• -BFS
  Agent runs breadth-first search.
• -DFS
  Agent runs depth-first search.
• -Astar
  Agent runs A* search.
                                  2
• -UCS
  Agent runs Uniform-cost search.
• -stats
  Your program should output the following information in addition to the final path:
     – The number of nodes expanded.
     – The length of the final path.
     – The ratio of nodes on the final path to nodes expanded.
     – The estimate of the total cost (as determined by the heuristic) from the start to the end (h(startstate)).
• -AltH
  (Only used with A* search.) Agent uses the heuristic you designed (rather than Manhattan distance).
• -AltC
  (Only used with A* or UCS.) Agent uses bottom-favoring cost function.
*/

char filename[256];
switch(argc)
{
     case 1: 
		printf("ERROR:  No arguments specified: \n");
		printf("USAGE: ./CS561A1.exe [FLAGS] FILENAME\n");
		exit(0);
		break;
     case 2: 	printf("ERROR: Only 1 arguments specified: \n");
		printf("USAGE: ./CS561A1.exe [FLAGS] FILENAME");
		printf("EG:    ./CS561A1.exe -BFS FILENAME\n");
		exit(0);
		break;

    default:   /*for(int i= 0; i< argc; i++)
			printf("argument%d = %s\n",i,argv[i]);
		*/
		int FLAG = 0;
		for(int i= 0; i< argc-1; i++)
		{
			if(!strcmp(argv[i],"-DFS")   || !strcmp(argv[i],"-dfs") )  {OPTIONDFS = 1;FLAG++;}
			if(!strcmp(argv[i],"-BFS")   || !strcmp(argv[i],"-bfs"))  {OPTIONBFS = 1;FLAG++;}
			if(!strcmp(argv[i],"-UCS")   || !strcmp(argv[i],"-ucs"))  {OPTIONUCS = 1;FLAG++;}
			if(!strcmp(argv[i],"-Astar") || !strcmp(argv[i],"-ASTAR") || !strcmp(argv[i],"-astar")){OPTIONASTAR = 1;FLAG++;}	

			if(!strcmp(argv[i],"-ALTH")  || !strcmp(argv[i],"-alth") ){ALTH=1;}
			if(!strcmp(argv[i],"-ALTC")  || !strcmp(argv[i],"-altc") ){ALTC=1;}
			if(!strcmp(argv[i],"-STATS") || !strcmp(argv[i],"-stats") ){STATS=1;}
		}   
		strcpy(filename,argv[argc-1]);
		
	if(DEBUG1) /* *******************************************************************************/
	{
		printf("OPTIONDFS = %d\n",OPTIONDFS);
		printf("OPTIONBFS = %d\n",OPTIONBFS);
		printf("OPTIONASTAR = %d\n",OPTIONASTAR);
		printf("OPTIONUCS = %d\n",OPTIONUCS);
		printf("ALTH   =%d\n",ALTH); //ALTH is purely for self defined Astar
		printf("ALTC   =%d\n",ALTC); //for Astar and UCS --bottom favouring
		printf("STATS  =%d\n",STATS);
		printf("filename:%s\n",filename);
	}

		
		if(FLAG==0 ||FLAG>1)
		{
			printf("ERROR: Only 1 arguments specified or two or more flags set at same time  \n");
			printf("USAGE: ./CS561A1.exe [FLAGS] FILENAME");
			printf("EG:    ./CS561A1.exe -BFS FILENAME\n");
			exit(0);	
		}
		//exit(0)
		break;
}

	
	totalelemsinfile = Readmatrixtextfile(filename);
	if(totalelemsinfile ==-1) exit(0);
	if(DEBUG1){displayfile();}
	initializeGridMatrix();
	initializeGridNodes();
	if(DEBUG1){displayGrid();}

	if(DEBUG1) /* *******************************************************************************/
	{	
		printf("grid width = %d\n",gridwidth);
		printf("grid height = %d\n", gridheight);
		printf("startpoint = (%d,%d)\n",startpointx,startpointy);
		printf("endpoint = (%d,%d)\n",endpointx,endpointy);
	}

	if(OPTIONBFS){bfs();return 1;}
	if(OPTIONDFS){dfs();return 1;}
	if(OPTIONUCS)
	{
		if(ALTC)
		{
			ucs_bottomfavouring();
		}
		else 
		{
			ucs();
		}
	}
	if(OPTIONASTAR)
	{	
		if(ALTH)
		{
			if(ALTC)
			{
				astar_selfdefinedheuristics_bottomfavor();
			}
			else
			{
				astar_selfdefinedheuristics_constgn();
			}
		}
		else
		{
			if(ALTC)
			{
				astarmanhattan_bottomfavor(); //bottom favouring g(n) and manhattan heuristics
			}
			else
			{
				astarmanhattan();            //constant cost g(n) and manhattan heuristics
			}	
		}
	}

	/*if ASTAR = 1           		run Manhattan const will run
	  if ASTAR = 1 and ALTC =1  		run Astar manhattan with bottomfavor
	  if UCS = 1             		run UCS const
	  if UCS = 1   and ALTC = 1 		run UCS with bottomfavor
	  if ASTAR = 1 and ALTH = 1    		run Selfdefined with const 
	  if ASTAR = 1 and ALTH = 1 and ALTC =1            run Selfdefined with bottom favouring
	  if BFS run bfs
	  if DFS run dFS
	*/ 
	return 1;
}

// self hue chet function

double ucost_constant(Node parent, Node neighbor)
{
	double cost = parent.cost + 1;
	//printf("(%d,%d) pcost = %f  (%d,%d)childcost = %f\n",parent.x,parent.y, parent.cost, neighbor.x, neighbor.y, cost);
	return cost;
}


double ucost_botfavor(Node parent, Node neighbor)
{
	double cost = parent.cost + (1 + (0.1 * neighbor.y));
	//printf("(%d,%d) pcost = %f  (%d,%d)childcost = %f\n",parent.x,parent.y, parent.cost, neighbor.x, neighbor.y, cost);
	return cost;	
}



double computeheuristic_manhattan(Node CurrentNode, Node GoalNode)
{
	double huecost = abs(GoalNode.x - CurrentNode.x ) + abs(GoalNode.y - CurrentNode.y);
	//printf("%f   ",huecost);
	return huecost;
}




//EUCLIDEAN DISTANCE

//Heuristic function should be such that it chooses a node that is guaranteed to be on the shortest path
//but if the next node u choose has a higher total cost it might take the previour branchs nodes for expansion insteaed of current expansion
//So to make sure that we get optimal path the heuristic should be admissible and yet have discrete difference between them
//MINE DOES THE FOLLOWING: sqrt( (Manhattan)*(Manhattan)/4  +  EUCLIDEAN*EUCLIDEAN)
double selfdefinedheuristics(Node CurrentNode, Node GoalNode)
{
	double finalcost = 0;
	double huecost = 0;
	double somemanhat = 0;

	double x2 = (GoalNode.x - CurrentNode.x)*(GoalNode.x - CurrentNode.x);   
        double y2 =  (GoalNode.y -CurrentNode.y)*(GoalNode.y -CurrentNode.y);   
	huecost = sqrt(x2+y2);	
	//huecost = sqrt( pow( (double)GoalNode.x - (double)CurrentNode.x ,2) + pow( (double)GoalNode.y - (double)CurrentNode.y,2));

	somemanhat = ((double)abs(GoalNode.x - CurrentNode.x )/2.0) + ((double)abs(GoalNode.y - CurrentNode.y)/2.0);
	
	finalcost = sqrt((somemanhat*somemanhat)+(huecost*huecost));
	//printf("%f   ",finalcost);
	return finalcost;
	//printf("%f   ",huecost);
	//return huecost;
	
}




bool operator > (const Node& GridNode1, const Node& GridNode2)
{
	return (GridNode1.cost + GridNode1.heuristic) > (GridNode2.cost + GridNode2.heuristic);
}




void	astar_selfdefinedheuristics_bottomfavor()
{
	printf("**************Astar SELF DEFINED with Bottom favouring G(n)************************\n");
	Priority_Queue Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;

	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.top();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true;numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    	GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; 
			double cost = ucost_botfavor(GridNode[k],GridNode[left]);
			double heucost = selfdefinedheuristics(GridNode[left],goalNode);
			GridNode[left].cost  = cost;
			GridNode[left].heuristic = heucost;
			Q.push(GridNode[left]); //numofnodestraversed++;	 //Note that overload operator will add heuristic and gn cost and add it to queue
			
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   	GridNode[right].visited = true;		
			GridNode[right].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[right]);
			double heucost = selfdefinedheuristics(GridNode[right],goalNode);
			GridNode[right].cost  = cost;
			GridNode[right].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[right]);	//numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{	GridNode[top].visited = true;		
			GridNode[top].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[top]);
			double heucost = selfdefinedheuristics(GridNode[top],goalNode);
			GridNode[top].cost  = cost;
			GridNode[top].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[top]);  		//numofnodestraversed++;	
			//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ 	GridNode[bottom].visited = true;           
		 	GridNode[bottom].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[bottom]);
			double heucost = selfdefinedheuristics(GridNode[bottom],goalNode);
			GridNode[bottom].cost  = cost;
			GridNode[bottom].heuristic = heucost;

			//printf("cost = %f\n",cost);
			Q.push(GridNode[bottom]);     //numofnodestraversed++;	
			// printf("k = %d   ", GridNode[bottom].parentindex);
		}		

		numofnodestraversed = numofnodestraversed+4;
	}//end of while
	
	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}

	if(STATS)
	{
		printf("\nEstimated cost by heuristic function : %f",selfdefinedheuristics(startNode,goalNode));
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}

}






void astarmanhattan_bottomfavor()
{

	printf("**************Astar Manhattan Heuristics  Bottom favouring G(n) and*************************\n");
	Priority_Queue Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;

	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.top();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true;numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    	GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; 
			double cost = ucost_botfavor(GridNode[k],GridNode[left]);
			double heucost = computeheuristic_manhattan(GridNode[left],goalNode);
			GridNode[left].cost  = cost;
			GridNode[left].heuristic = heucost;
			Q.push(GridNode[left]); // numofnodestraversed++;	 //Note that overload operator will add heuristic and gn cost and add it to queue
			
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   	GridNode[right].visited = true;		
			GridNode[right].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[right]);
			double heucost = computeheuristic_manhattan(GridNode[right],goalNode);
			GridNode[right].cost  = cost;
			GridNode[right].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[right]);	//numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{	GridNode[top].visited = true;		
			GridNode[top].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[top]);
			double heucost = computeheuristic_manhattan(GridNode[top],goalNode);
			GridNode[top].cost  = cost;
			GridNode[top].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[top]);  		//numofnodestraversed++;	
			//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ 	GridNode[bottom].visited = true;           
		 	GridNode[bottom].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[bottom]);
			double heucost = computeheuristic_manhattan(GridNode[bottom],goalNode);
			GridNode[bottom].cost  = cost;
			GridNode[bottom].heuristic = heucost;

			//printf("cost = %f\n",cost);
			Q.push(GridNode[bottom]);   //  numofnodestraversed++;	
			// printf("k = %d   ", GridNode[bottom].parentindex);
		}		

numofnodestraversed = numofnodestraversed+4;
	}//end of while
	


	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0); 
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	if(STATS)
	{
		printf("\nEstimated cost by heuristic function : %f",computeheuristic_manhattan(startNode,goalNode));
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}
}


























void 	astar_selfdefinedheuristics_constgn()
{
	printf("**************Astar SELF DEFINED With constant GN Cost ************************\n");
	Priority_Queue Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;

	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.top();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true;numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    	GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; 
			double cost = ucost_constant(GridNode[k],GridNode[left]);
			double heucost = selfdefinedheuristics(GridNode[left],goalNode);
			GridNode[left].cost  = cost;
			GridNode[left].heuristic = heucost;
			Q.push(GridNode[left]); //numofnodestraversed++;	 //Note that overload operator will add heuristic and gn cost and add it to queue
			
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   	GridNode[right].visited = true;		
			GridNode[right].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[right]);
			double heucost = selfdefinedheuristics(GridNode[right],goalNode);
			GridNode[right].cost  = cost;
			GridNode[right].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[right]);	//numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{	GridNode[top].visited = true;		
			GridNode[top].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[top]);
			double heucost = selfdefinedheuristics(GridNode[top],goalNode);
			GridNode[top].cost  = cost;
			GridNode[top].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[top]);  		//numofnodestraversed++;	
			//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ 	GridNode[bottom].visited = true;           
		 	GridNode[bottom].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[bottom]);
			double heucost = selfdefinedheuristics(GridNode[bottom],goalNode);
			GridNode[bottom].cost  = cost;
			GridNode[bottom].heuristic = heucost;

			//printf("cost = %f\n",cost);
			Q.push(GridNode[bottom]);    // numofnodestraversed++;	
			// printf("k = %d   ", GridNode[bottom].parentindex);
		}		

		numofnodestraversed= numofnodestraversed+4;
	}//end of while
	



	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	if(STATS)
	{
		printf("\nEstimated cost by heuristic function : %f",selfdefinedheuristics(startNode,goalNode));
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}

}









































void astarmanhattan()
{

	printf("**************A star manhattan-heuristics Constant Gn *************************\n");
	Priority_Queue Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;

	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.top();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true; numofnodestraversed++; break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    	GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; 
			double cost = ucost_constant(GridNode[k],GridNode[left]);
			double heucost = computeheuristic_manhattan(GridNode[left],goalNode);
			GridNode[left].cost  = cost;
			GridNode[left].heuristic = heucost;
			Q.push(GridNode[left]); //numofnodestraversed++;	 //Note that overload operator will add heuristic and gn cost and add it to queue
			
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   	GridNode[right].visited = true;		
			GridNode[right].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[right]);
			double heucost = computeheuristic_manhattan(GridNode[right],goalNode);
			GridNode[right].cost  = cost;
			GridNode[right].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[right]);  //	numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{	GridNode[top].visited = true;		
			GridNode[top].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[top]);
			double heucost = computeheuristic_manhattan(GridNode[top],goalNode);
			GridNode[top].cost  = cost;
			GridNode[top].heuristic = heucost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[top]);  	//	numofnodestraversed++;	
			//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ 	GridNode[bottom].visited = true;           
		 	GridNode[bottom].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[bottom]);
			double heucost = computeheuristic_manhattan(GridNode[bottom],goalNode);
			GridNode[bottom].cost  = cost;
			GridNode[bottom].heuristic = heucost;

			//printf("cost = %f\n",cost);
			Q.push(GridNode[bottom]);     //numofnodestraversed++;	
			// printf("k = %d   ", GridNode[bottom].parentindex);
		}		

		numofnodestraversed = numofnodestraversed+4;
	}//end of while
	




	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	
	if(STATS)
	{
		printf("\nEstimated cost by heuristic function : %f",computeheuristic_manhattan(startNode,goalNode));
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}
		
}

















void ucs_bottomfavouring()
{

	printf("**************Uniform Cost  Search  Bottom favouring *************************\n");
	Priority_Queue Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;

	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.top();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true;numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    	GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; 
			double cost = ucost_botfavor(GridNode[k],GridNode[left]);
			GridNode[left].cost  = cost;
			Q.push(GridNode[left]);         //numofnodestraversed++;		
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   	GridNode[right].visited = true;		
			GridNode[right].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[right]);
			GridNode[right].cost = cost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[right]);	//numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{	GridNode[top].visited = true;		
			GridNode[top].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[top]);
			GridNode[top].cost = cost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[top]);    	//numofnodestraversed++;	
			//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ 	GridNode[bottom].visited = true;           
		 	GridNode[bottom].parentindex = k;
			double cost = ucost_botfavor(GridNode[k],GridNode[bottom]);
			GridNode[bottom].cost  = cost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[bottom]);     //numofnodestraversed++;	
		// printf("k = %d   ", GridNode[bottom].parentindex);
		}		

		numofnodestraversed= numofnodestraversed+4;
	}//end of while
	






	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	if(STATS)
	{
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}
}





void ucs()
{
	printf("**************Uniform Cost  Search - constant cost function*************************\n");
	Priority_Queue Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;

	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.top();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true;numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    	GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; 
			double cost = ucost_constant(GridNode[k],GridNode[left]);
			GridNode[left].cost  = cost;
			Q.push(GridNode[left]); //numofnodestraversed++;	
			
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   	GridNode[right].visited = true;		
			GridNode[right].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[right]);
			GridNode[right].cost = cost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[right]);   //	numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{	GridNode[top].visited = true;		
			GridNode[top].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[top]);
			GridNode[top].cost = cost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[top]);  	//	numofnodestraversed++;	
			//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ 	GridNode[bottom].visited = true;           
		 	GridNode[bottom].parentindex = k;
			double cost = ucost_constant(GridNode[k],GridNode[bottom]);
			GridNode[bottom].cost  = cost;
			//printf("cost = %f\n",cost);
			Q.push(GridNode[bottom]);  //   numofnodestraversed++;	
		// printf("k = %d   ", GridNode[bottom].parentindex);
		}		

	numofnodestraversed = numofnodestraversed+4;
	}//end of while



	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	if(STATS)
	{
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}
}













void dfs()
{
	printf("**************Depth First Search*************************\n");
	stack<Node> Stack;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;


	//k = GridMatrix[x][y] will give u GridNode[k]

	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Stack.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Stack.empty())
	{
		bool wasexpanded= false;
		temp = Stack.top();
		Stack.pop();	
		
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true;numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; Stack.push(GridNode[left]); //numofnodestraversed++;	
		//printf("k = %d   ",GridNode[left].parentindex); 
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   GridNode[right].visited = true;		
		GridNode[right].parentindex = k;Stack.push(GridNode[right]);  //numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{GridNode[top].visited = true;		
		GridNode[top].parentindex = k;Stack.push(GridNode[top]); // numofnodestraversed++;	
		//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ GridNode[bottom].visited = true;           
		 GridNode[bottom].parentindex = k;Stack.push(GridNode[bottom]); //numofnodestraversed++;	
		// printf("k = %d   ", GridNode[bottom].parentindex);
		}
		numofnodestraversed = numofnodestraversed+4;			
	}//end of while
	
	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n NO SOLUTION",goalNode.x,goalNode.y);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c \n ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	if(STATS)
	{
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);	
	}
}


















void bfs()
{
	printf("**************Breadth First Search*************************\n");
	queue<Node> Q;
	int startindex = getstartNodeindex();
	int goalindex = getgoalNodeindex();
	Node temp;
	bool flag = false;


	//k = GridMatrix[x][y] will give u GridNode[k]

	
	int numofnodestraversed = 0;
	int numofnodesonpath = 0;

	
	GridNode[startindex].visited = true;
	Q.push(GridNode[startindex]);
	numofnodestraversed++;	
	//displayXY(startNode);printf("\n");
	
	if(DEBUG1){	printf("*****************ALL NODES EXAMINED**************************\n");}
	while(!Q.empty())
	{
		bool wasexpanded= false;
		temp = Q.front();
		Q.pop();	
		
		int k = GridMatrix[temp.x][temp.y];   // k stores the parent node index 
				
		if(DEBUG1){displayXY(temp);printf("\n");}

		if(temp.x==goalNode.x && temp.y==goalNode.y ){ flag = true; numofnodestraversed++;break; }		    //check if the node is goal node

				
		int left    = getindex( (temp.x)-1, temp.y);  
		if(left!=-1&& !(GridNode[left].visited) &&  GridNode[left].type!='N')
		{    GridNode[left].visited = true; 	
			GridNode[left].parentindex = k; Q.push(GridNode[left]); //numofnodestraversed++;	
		//printf("k = %d   ",GridNode[left].parentindex); 
		}

		int right   = getindex((temp.x)+1, temp.y);  
		if(right!=-1&& !(GridNode[right].visited)&& GridNode[right].type!='N') 
		{   GridNode[right].visited = true;		
		GridNode[right].parentindex = k;Q.push(GridNode[right]);	//numofnodestraversed++;	
		//printf("k = %d   ",GridNode[right].parentindex);
		}

		int top     = getindex(temp.x,   (temp.y)+1);
		if(top!=-1     && !(GridNode[top].visited) && GridNode[top].type!='N') 
 		{GridNode[top].visited = true;		
		GridNode[top].parentindex = k;Q.push(GridNode[top]);  		//numofnodestraversed++;	
		//printf("k = %d   ",GridNode[top].parentindex);
		}

		int bottom  = getindex(temp.x,   (temp.y)-1);
		if(bottom!=-1  && !(GridNode[bottom].visited) && GridNode[bottom].type!='N')
		{ GridNode[bottom].visited = true;           
		 GridNode[bottom].parentindex = k;Q.push(GridNode[bottom]);     //numofnodestraversed++;	
		// printf("k = %d   ", GridNode[bottom].parentindex);
		}

	numofnodestraversed= numofnodestraversed+4;
					
	}//end of while
	
	if(!flag)
	{
		printf("GOAL NODE (%d,%d) NOT FOUND\n  NO SOLUTION  \n",goalNode.x,goalNode.y); exit(0);
	}
	else
	{	
		if(DEBUG1){printf("GOAL NODE FOUND: %d,%d : type = %c\n  ",temp.x,temp.y,temp.type);}
		
		Node P = GridNode[goalindex];
		
		stack<Node> S;
		S.push(GridNode[goalindex]);numofnodesonpath++;
		while(1)
		{	
			
			if(P.x==startNode.x && P.y==startNode.y){GridNode[startindex].type='S';break;}
			int k = P.parentindex;
			if(k!=-100)
			{
				//printf("(%d,%d)\n",GridNode[k].x,GridNode[k].y);	
				GridNode[k].type='P';
				S.push(GridNode[k]);
				P = GridNode[k];
				numofnodesonpath++;
			}
		}
		if(DEBUG1){displayvisited();displaytype();}
		//S.push(GridNode[startindex]);
		//printf("(%d,%d) ",startNode.x,startNode.y);
		printf("PATH:\n");
		while(!S.empty())
		{

			Node N = S.top();
			S.pop();
			printf("(%d,%d)\n",N.x,N.y);
		}


	}
	if(STATS)
	{
		printf("\nNumber of nodes expanded : %d\n",numofnodestraversed);
		printf("Number of nodes on path  : %d\n",numofnodesonpath);
		printf("Ratio : %f\n",(float)numofnodesonpath/(float)numofnodestraversed);
	}
}


int getstartNodeindex()
{
			int x = testmatrix[2];
			int y = testmatrix[3];
			int k = GridMatrix[x][y];
return k;			

}

int getgoalNodeindex()
{
			int x = testmatrix[4];
			int y = testmatrix[5];
			int k = GridMatrix[x][y];
return k;
}



void initializeGridMatrix()
{
       int k=0;
	//cout << "Gridwidth = "<<gridwidth<<"gridheight"<<gridheight;
	for(int i =0; i<gridheight; i++)
	{
		for(int j=0;j<gridwidth; j++)
		{
			GridMatrix[j][i] = k;
			k++;
		}

	}
}






void initializeGridNodes()
{

	for(int i = 0; i<gridwidth*gridheight; i++)
	{
		GridNode[i].type = '*';
		GridNode[i].visited = false;
		GridNode[i].parentindex = -100;
		GridNode[i].cost = 0;
		GridNode[i].heuristic = 0;
	}

	int k = 0;
	for(int i =0; i< gridheight; i++)
	{
		for(int j=0;j<gridwidth; j++)
		{
		
			GridNode[k].x = j;
			GridNode[k].y = i;
                        k++; 
		}
	}
	for(int i = 6; i<totalelemsinfile;i=i+2)
	{		
		if(testmatrix[i]!=-1 && testmatrix[i+1]!=-1)
		{
			int x = testmatrix[i];
			int y = testmatrix[i+1];
			int k = GridMatrix[x][y];
		      //printf("%d %d %d \n", x, y, k);
		        GridNode[k].x = x;
		      	GridNode[k].y = y;
		      	GridNode[k].type = 'N';
			
		}
	}

			//set start and end node:	
			int x = testmatrix[2];
			int y = testmatrix[3];
			k = GridMatrix[x][y];
		      //printf("%d %d %d \n", x, y, k);
		        GridNode[k].x = x;
		      	GridNode[k].y = y;
		      	GridNode[k].type = 'S';
			startNode = GridNode[k];
			//GridNode[k].visited = true;
			
			x = testmatrix[4];
			y = testmatrix[5];
			k = GridMatrix[x][y];
		      //printf("%d %d %d \n", x, y, k);
		        GridNode[k].x = x;
		      	GridNode[k].y = y;
		      	GridNode[k].type = 'E';
			goalNode = GridNode[k];
			//GridNode[k].visited = true;

			//printf("startnode = %d,%d %c %d",startNode.x,startNode.y,startNode.type,startNode.visited);
}
//initializegridNodes function ends 







void displayGrid()
{
  
     	int k=0;
	printf("**************************************************\n");
	for(int i = gridheight-1; i>=0; i--)
	{
		for(int j=0;j<gridwidth; j++)
		{
			printf("%4d",GridMatrix[j][i]);	
		}
		printf("\n");
	}

	printf("**************************************************\n");
	for(int i = gridheight-1; i>=0; i--)
	{
		for(int j=0;j<gridwidth; j++)
		{
			displayXY(GridNode[GridMatrix[j][i]]);
		}
		printf("\n");
	}

	printf("**************************************************\n");
	for(int i = gridheight-1; i>=0; i--)
	{
		for(int j=0;j<gridwidth; j++)
		{
			displayNodetype(GridNode[GridMatrix[j][i]]);
		}
		printf("\n");
	}
	printf("******************** VISITED NOT VISITED*****************************\n");
	for(int i = gridheight-1; i>=0; i--)
	{
		for(int j=0;j<gridwidth; j++)
		{
			int k = GridMatrix[j][i];
			printf("%4d",GridNode[k].visited);
		}
		printf("\n");
	}
}



void displayXY(Node N)
{
	printf("(%2d,%2d) ",N.x,N.y);

}

void displaytype()
{
	printf("**************************************************\n");
	for(int i = gridheight-1; i>=0; i--)
	{
		for(int j=0;j<gridwidth; j++)
		{
			displayNodetype(GridNode[GridMatrix[j][i]]);
		}
		printf("\n");
	}
}

void displayvisited()
{
	printf("******************** 0 is not visited 1 is visited*****************************\n");
	for(int i = gridheight-1; i>=0; i--)
	{
		for(int j=0;j<gridwidth; j++)
		{
			int k = GridMatrix[j][i];
			printf("%4d",GridNode[k].visited);
		}
		printf("\n");
	}
}


void displayNodetype(Node N)
{
	printf("%4c ",N.type);
}






int getindex(int x, int y)
{
if(x>MAXGRIDSIZE || x <0 ||y<0 || y>MAXGRIDSIZE)
	return -1;
else
	return GridMatrix[x][y];
}





void displayfile()
{
	printf("grid width = %d\n",gridwidth);
	printf("grid height = %d\n", gridheight);
	printf("startpoint = (%d,%d)\n",startpointx,startpointy);
	printf("grid height = (%d,%d)\n",endpointx,endpointy);
	printf("**************Contents of the file **************************\n");
		for(int i=0; i<totalelemsinfile; i=i+2)
		{
			printf("%2d  ",testmatrix[i]);
			printf("%2d  \n",testmatrix[i+1]);
		}


}

int Readmatrixtextfile(char *filename)
{
	  FILE * fd;
	  int ch;
	  fd = fopen (filename,"r");
	  if (fd==NULL)
	  {
		  printf("bad file name\n");
		  
		  exit(0);
	  }
	 	  // printf("\ntestmatrix FILE : %s",filename);
		  char number[100000];
		  //int testmatrix[10000];
		  int i=0;
		  int j=0;

			while(1)
			{
				ch=fgetc(fd);
				if(ch==EOF)
				{
					break;
				} 
				if(ch==32 || ch =='\n' ||ch==44)  //32 is asci for space --look for spaces and newlines or comma 
				{	
					continue;
				}
				if( ch!=32 || ch !='\n' || ch!=EOF ||ch!=44)      //if its a number and not a space or newline or EOF then its a number
				{
		       			i=0;
					number[i++]=(char)ch;	//put the num to character array
					ch=fgetc(fd);
					while(1) //collect all numbers till you hit a space or a newline or comma afterwhich append a '\0'
					{
						if((ch=='\n')|| (ch==32) || (ch==EOF) || (ch==44)){break;}
						number[i++]=(char)ch; ch=fgetc(fd);				
					}
					if(ch==EOF)
					{	
						number[i]='\0';testmatrix[j] =atoi(number);  // we break the while loop if its eof							
									
						if(j==0) 
						{
							gridwidth = testmatrix[j]; 
							if(gridwidth>MAXGRIDSIZE || gridwidth <MINGRIDSIZE) 
							{
								printf("Error:Gridwidth:%d should be [2,30]\n",gridwidth);exit(0);
							}
						}
						if(j==1) 
						{
						 gridheight = testmatrix[j];
						 if(gridheight>MAXGRIDSIZE || gridheight <MINGRIDSIZE) 
							{
								printf("Error:Gridheight:%d should be [2,30]\n",gridheight);
								exit(0);
							}
						}
						if(j==2) startpointx = testmatrix[j];
						if(j==3) startpointy = testmatrix[j];
						if(j==4) endpointx = testmatrix[j];
						if(j==5) endpointy = testmatrix[j];



						if(testmatrix[j]!=-1) //error condition to check if values are in range. between 2 and 30
							{
							if(testmatrix[j]>MAXGRIDSIZE || testmatrix[j] <0)
								printf("Error:wrong Wire value:%d should be [2,30]\n",testmatrix[j]);exit(0);
							}
						j++;
						break;
					}
					else 
					{
						if(i>0)
						{
							number[i]='\0';testmatrix[j] =atoi(number); i=0;
							if(j==0) 
							{
								gridwidth = testmatrix[j]; 
								if(gridwidth>MAXGRIDSIZE || gridwidth <MINGRIDSIZE) 
								{
									printf("Error:Gridwidth:%d should be [2,30]\n",gridwidth);exit(0);
								}
							}
							if(j==1) 
							{
								gridheight = testmatrix[j];
								if(gridheight>MAXGRIDSIZE || gridheight <MINGRIDSIZE) 
								{
									printf("Error:Gridheight:%d should be [2,30]\n",gridheight);exit(0);
								}
							}
							if(j==2) startpointx = testmatrix[j];
							if(j==3) startpointy = testmatrix[j];
							if(j==4) endpointx = testmatrix[j];
							if(j==5) endpointy = testmatrix[j];
							if(testmatrix[j]!=-1) //error condition to check if values are in range. between 2 and 30
							{
									if(testmatrix[j]>MAXGRIDSIZE || testmatrix[j] < 0)
										{printf("Error:wrong Wire value:%d should be [2,30]\n",testmatrix[j]);exit(0);}
							}
							j++;//else we continue
						}
					} //else its always a space or a newline}
				}

			}
	fclose(fd);

	if(j==0) {printf("Error in Input file:Empty file \n"); exit(0);}
	if(j<=2) {printf("Error in Input file: Start point and end point not specified in File \n"); exit(0);}
	if(j<=4) {printf("Warning:This is an empty grid with no wires\n");}
	
	return j;
}
