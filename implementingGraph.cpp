#include<iostream>
#include<unordered_map>
#include<list>
#include<queue>
using namespace std;

class Graph{
    public:
    unordered_map<int,list<int>> adjList;
    //can be used vector too for mapping

    void addEdge(int u,int v,bool direction){
        //direction=0-> undirected graph edge
        //direction=1->directed grafh edge
        if(direction==0){
            adjList[u].push_back(v);
            adjList[v].push_back(u);
        }
        else{
            adjList[u].push_back(v);
        }
    }
    void printAdjList(int n){

        for(int i=0;i<n;i++){
            cout<<i<<":";
            cout<<"{";
            list<int> temp = adjList[i];
            for(auto j:temp){
                cout<<j<<",";
            }
            cout<<"}"<<endl;
        }
        
    }
    bool cycleDetectedUndirectedGraph(int src,unordered_map<int,bool> visited){
        queue<int> q;
        unordered_map<int,int> parent;
        
        
        q.push(src);
        visited[src]=true;
        parent[src]=-1;

        //main logic
        while(!q.empty()){
            int frontNode = q.front();
            q.pop();

            for(auto nbr:adjList[frontNode]){
                if(!visited[nbr]){
                    q.push(nbr);
                    visited[nbr] = true;
                    parent[nbr] = frontNode;

                }
                else if(visited[nbr]==true && nbr!=parent[frontNode]){
                    //cycle present 
                    return true;
                }

            }
        }
        return false;
    }
};
class weightedGraph{
    public:
    unordered_map<int,list<pair<int,int>>> adjList;
    //can be used vector too for mapping

    void addEdge(int u,int v,int weight,bool direction){
        //direction=0-> undirected graph edge
        //direction=0->directed grafh edge
        if(direction==0){
            adjList[u].push_back({v,weight});
            adjList[v].push_back({u,weight});
        }
        else{
            adjList[u].push_back({v,weight});
        }
    }
    void printAdjList(int n){

        for(int i=0;i<n;i++){
            cout<<i<<":";
            cout<<"{";
            list<pair<int,int>> temp = adjList[i];
            for(auto j:temp){
                cout<<"("<<j.first<<","<<j.second<<"),";
            }
            cout<<"}"<<endl;
        }
        
    }
    void bfsTraversal(int src){
        queue<int> q;
        unordered_map<int,bool> visited;
        cout<<"BFS: ";

        q.push(src);
        visited[src] = true;

        while(!q.empty()){
            int frontNode = q.front();
            cout<< frontNode<<",";
            q.pop();

            for(auto neighbor:adjList[frontNode]){
                //negihbour ->pair
                //first->node
                //second->weight
                int node = neighbor.first;
                //int weight = neighbor.second;
                if(!visited[node]){
                    q.push(node);
                    visited[node] = true;
                }
            }
        }
        cout<<endl;
    }
    void dfs(int n){
        //int src = 0;
        unordered_map<int,bool>visited;
        for(int src=0;src<n;src++){
            if(!visited[src])
                dfsHelper(src,visited);
        }
    }

    void dfsHelper(int src,unordered_map<int,bool>& visited){
        visited[src] = true;
        cout<<src<<",";

        for(auto neighbour:adjList[src]){
            int node = neighbour.first;
            if(!visited[node]){
                dfsHelper(node,visited);
            }
        }
    }
};
int main(){
    Graph g;
    int n = 7;
    g.addEdge(0,1,1);
    g.addEdge(0,2,1);
    g.addEdge(1,3,1);
    g.addEdge(2,3,1);

    g.printAdjList(4);
    bool ans = false;
    unordered_map<int,bool> visited;
    for(int i=0;i<4;i++){
        if(!visited[i]){
            bool ans = g.cycleDetectedUndirectedGraph(i,visited);
            if(ans == true){
                break; 
            }
        }
    }
    if(ans == true){
        cout<<"CYCLE FOUND"<<endl;
    }
    else{
        cout<<"CYCLE NOT FOUND"<<endl;
    }

    weightedGraph wg;

    wg.addEdge(0,3,3,1);
    wg.addEdge(0,5,4,1);
    wg.addEdge(0,2,16,1);
    wg.addEdge(3,5,1,1);
    wg.addEdge(2,5,16,1);
    wg.addEdge(5,4,16,1);
    wg.addEdge(5,6,16,1);
    wg.addEdge(4,1,16,1);
    wg.addEdge(6,1,16,1);

    wg.printAdjList(n);
    
    wg.bfsTraversal(0);
    cout<<"DFS:";
    wg.dfs(7);



}