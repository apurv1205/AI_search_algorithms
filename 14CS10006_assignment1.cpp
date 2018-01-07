#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <set>
#include <time.h>
#include <math.h>

using namespace std;

set<vector<vector<char> > > visited;
// botton, left, top, right for finding child nodes
int row[4] = { 1, 0, -1, 0 };
int col[4] = { 0, -1, 0, 1 };
//to store the visited configurations, if a repeated configuration is observed, ignore
int final[3][3] = { {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 0} };

//Structure of the node to be used in space search
struct Node
{
    vector<vector<char> > conf;   // stores configuration of the tiles in each node
    int x, y;                     // stores blank tile(0) cordinates
    int h;                        // stores the heuristic cost to reach the goal state
    int g;                        // stores the actual cost to reach the tile from the start tile
    Node* parent;                 // stores parent node of current node to print the sequence of steps or the path in the space search tree in the end
};
 
bool isGoal(vector<vector<char> > conf) {
    for(int i=0;i<3;i++) {
        for (int j=0;j<3;j++) {
            if (conf[i][j] != final[i][j]) return false;
        }
    }
    return true;
}

//mthod defines the heuristic cost calculation method
int calculateHeuristicCost(vector<vector<char> > initial, int method)
{
    if (method==0) {            // h=0 : Bredth first search
        return 0;
    } else if (method==1) {     // h=hamming distance : calculates the the number of non blank misplaced tiles
        int count=0;
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
            if (initial[i][j] && initial[i][j] != final[i][j])
               count++;
        return count;  
    } else if (method==2) {     // h = manhattan distance : manhattan distance between the non blank tiles between current and goal configuration
        int dist=0;
        for(int i=0;i<3;i++)
        { for(int j=0;j<3;j++)
            { if(initial[i][j]!=0)
                { for(int k=0;k<3;k++)
                    { for(int l=0;l<3;l++)
                        { if(initial[i][j]==final[k][l]) dist+=abs(i-k)+abs(j-l); 
        }   }   }   }   }
        return dist;
    } else {                    // h = euclidean distance : euclidean distance between the non blank tiles between current and goal configuration
        float dist=0.0;
        for(int i=0;i<3;i++)
        { for(int j=0;j<3;j++)
            { if(initial[i][j]!=0)
                { for(int k=0;k<3;k++)
                    { for(int l=0;l<3;l++)
                        { if(initial[i][j]==final[k][l]) dist+=sqrt(pow(i-k,2)+pow(j-l,2)); 
        }   }   }   }   }
        return (int)dist;
    }
}
 
// print path from initial state to goal state
void printPath(Node* node)
{
    int count = 0;
    while(node != NULL) {
        count++;
        node = node->parent;
    }
    cout<<setw(30)<<count;
}
 
// Function to expand or create a new node
Node* expandNode(vector<vector<char> > conf, int x, int y, int newX,
              int newY, int g, Node* parent)
{
    Node* node = new Node;
    node->parent = parent;
    node->conf = conf;
    // move blank tile by 1 postion
    swap(node->conf[x][y], node->conf[newX][newY]);
    node->h = INT_MAX;
    node->g = g;
    // update new blank tile cordinates
    node->x = newX;
    node->y = newY;
    return node;
}

// Comparison object to be used to order the heap
struct comp2
{   bool operator()(const Node* lhs, const Node* rhs) const{
        return (lhs->h + lhs->g) > (rhs->h + rhs->g);
    }
};
 
// solve 8 puzzle using A* search
void AStar(vector<vector<char> > initial, int x, int y, int method)
{
    visited.clear();
    clock_t t1,t2;
    t1=clock();
    priority_queue<Node*, vector<Node*>, comp2> Open;       // Create a priority queue to store open nodes of search tree which are to be expanded
    vector<Node*> Closed;                                   // To store the closed nodes or a list of all the nodes expanded till now

    Node* root = expandNode(initial, x, y, x, y, 0, NULL);  // create a root node and calculate its h cost
    root->h = calculateHeuristicCost(initial, method);
 
    // Add root to list of open nodes;
    Open.push(root);
    visited.insert(root->conf);
    // Finds an open node with least h,
    // add its childrens to list of open nodes, the current node to closed list and
    // finally deletes it from the list.
    while (!Open.empty())
    {
        // Find a open node with least esticonfed h
        Node* min = Open.top();
 
        // if min's configuration is goal state
        if ( (min->h == 0 && method > 0 ) || isGoal(min->conf) )
        {
            // print the path from root to destination;
            printPath(min);
            cout<<setw(40)<<Closed.size();
            t2=clock();
            float diff((float)t2-(float)t1);
            cout<<setw(50)<<diff/CLOCKS_PER_SEC;
            return;
        }

        // The found node is deleted from the list of open nodes and added to close list
        Open.pop();
        Closed.push_back(min);
 
        // do for each child of min node
        for (int i = 0; i < 4; i++)
        {
            Node* child = NULL;
            if (min->x + row[i] >= 0 && min->x + row[i] < 3 && min->y + col[i] >= 0 && min->y + col[i] < 3)
            {
                child = expandNode(min->conf, min->x,
                              min->y, min->x + row[i],
                              min->y + col[i],
                              min->g + 1, min);
            } else continue;
            set<vector<vector<char> > >::iterator it = visited.find(child->conf);
            if (it == visited.end()) {
            child->h = calculateHeuristicCost(child->conf, method);

            // Add child to list of live nodes
            Open.push(child);
            visited.insert(child->conf);
            }
        }
    }
    cout<<setw(30)<<"No Solution!";
    cout<<setw(40)<<Closed.size();
    t2=clock();
    float diff((float)t2-(float)t1);
    cout<<setw(50)<<diff/CLOCKS_PER_SEC;
    return;
}
int lastExpandedCount = 0;

//Helper recursive function for IDAStar algorithm, prunes the branches with higher costs than bound.
//if a an already searched node is observed in an iteration, it is skipped
int search(vector<Node*> path, set<vector<vector<char> > > path1, int g, int bound, int method) {
    Node* node = path.back();
    int f = g + calculateHeuristicCost(node->conf, method);
    if (f>bound) return f;
    if (isGoal(node->conf)) {
        printPath(node);
        return -1;
    }
    int min = INT_MAX;
    // do for each child of min node
    for (int i = 0; i < 4; i++)
    {
        Node* child = NULL;
        if (node->x + row[i] >= 0 && node->x + row[i] < 3 && node->y + col[i] >= 0 && node->y + col[i] < 3)
        {
            child = expandNode(node->conf, node->x,
                          node->y, node->x + row[i],
                          node->y + col[i],
                          node->g + 1, node);
        } else continue;
        set<vector<vector<char> > >::iterator it = path1.find(child->conf);
        if (it == path1.end()) {
            lastExpandedCount++;
            path.push_back(child);
            pair< set<vector<vector<char> > >::iterator,bool> it1 = path1.insert(child->conf);
            int t = search(path, path1, g+1 , bound, method);
            if (t==-1) return -1;
            if (t<min) min = t;
            path.pop_back();
            path1.erase(it1.first);
        }
    }
    return min;
}

//IDAStar search
//visited stores the nodes expanded in the most recent iteration
void IDAStar(vector<vector<char> > initial, int x, int y, int method){
    clock_t t1,t2;
    t1=clock();
    Node* root = expandNode(initial, x, y, x, y, 0, NULL);  // create a root node and calculate its h cost
    int bound = calculateHeuristicCost(root->conf, method);
    vector<Node*> path;
    set<vector<vector<char> > > path1;
    path.push_back(root);
    path1.insert(root->conf);
    while(1) {
        lastExpandedCount = 0;
        int t = search(path, path1, 0 , bound, method);
        if (t == -1) {
            cout<<setw(40)<<lastExpandedCount;
            t2=clock();
            float diff((float)t2-(float)t1);
            cout<<setw(50)<<diff/CLOCKS_PER_SEC;
            return;
        } else if (t==INT_MAX) {
            cout<<setw(30)<<"No Solution!";
            cout<<setw(40)<<lastExpandedCount;
            t2=clock();
            float diff((float)t2-(float)t1);
            cout<<setw(50)<<diff/CLOCKS_PER_SEC;
            return;
        }
        bound = t;
    }
}

int main()
{   
    // Initial configuration
    // Value 0 is used for empty space
    int x = 0, y = 0;
    int arr[9] = {1,3,2,5,4,0,6,8,7};
    int cnt = 13;
    while(cnt--) {
        vector<vector<char> > initial(3, vector<char> (3,0));
        printf("\n\n*****************************************************************************************************************************************************\n\n+++INPUT :\n\n");
        for (int i=0;i<initial.size();i++) {
            for (int j=0;j<initial.size();j++) {
                int temp=0;
                temp = arr[i*3+j];
                if(temp==0) {
                    x=i;
                    y=j;
                }
                printf(" %d",temp);
                initial[i][j] = temp;
            }
            printf("\n");
        }
        int inv_count = 0;
        for (int i = 0; i < 9 - 1; i++)
            for (int j = i+1; j < 9; j++)
                 if (arr[j] && arr[i] &&  arr[i] > arr[j])
                      inv_count++;
        if (inv_count%2!=0) {
            printf("\n\nNot Solvable puzzle, not running the A* and IDA* algorithms !\n\n");
            random_shuffle(&arr[0], &arr[8]);
            continue;
        }
        printf("\n\n+++A star algorithm :\n");
        printf("\nHeuristic Cost Method\t\t\tMin number of moves\t\t\tNumber of nodes expanded\t\t\tRunning time (in s)\n\n");
        cout<<setw(26)<<"H = 0 (BFS)";
        AStar(initial, x, y, 0);
        cout<<setw(26)<<"\n  Sum of hamming distances";
        AStar(initial, x, y, 1);
        cout<<setw(26)<<"\nSum of manhattan distances";
        AStar(initial, x, y, 2);
        cout<<setw(26)<<"\nSum of euclidean distances";
        AStar(initial, x, y, 3);

        printf("\n\n\n\n+++IDA star algorithm :\n");
        printf("\nHeuristic Cost Method\t\t\tMin number of moves\t\t\tNumber of nodes expanded\t\t\tRunning time (in s)\n\n");
        cout<<setw(26)<<"H = 0 (BFS)";
        IDAStar(initial, x, y, 0);
        cout<<setw(26)<<"\n  Sum of hamming distances";
        IDAStar(initial, x, y, 1);
        cout<<setw(26)<<"\nSum of manhattan distances";
        IDAStar(initial, x, y, 2);
        cout<<setw(26)<<"\nSum of euclidean distances";
        IDAStar(initial, x, y, 3);
        srand(time(0));
        random_shuffle(&arr[0], &arr[8]);
    }
    return 0;
}