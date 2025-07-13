#include<iostream>
#include<unordered_map>
#include<list>
#include<queue>
#include<algorithm>
#include<stack>
#include<limits.h>
using namespace std;
class Graph{
    public:
        unordered_map<int,list<int>> adj;
        unordered_map<int,list<pair<int,int>>> adjWt;
        void addEdge(int u,int v,bool direction){
            //direction=0->unidirected
            //direction=1->directed
            if(direction==0){
                adj[u].push_back(v);
                adj[v].push_back(u);
            }
            else{
                adj[u].push_back(v);
            }
        }
        void addEdgeWeight(int u,int v,int wt,bool direction){
            //direction=0->unidirected
            //direction=1->directed
            if(direction==0){
                adjWt[u].push_back({v,wt});
                adjWt[v].push_back({u,wt});
            }
            else{
                adjWt[u].push_back({v,wt});
            }
        }
        void printAdjList(int n){
            for(int i=0;i<n;i++){
                cout<<i<<":";
                for(auto j:adj[i]){
                    cout<<j<<",";
                }
                cout<<endl;
            }
        }
        void shortestDestination(int src,int destination){
            unordered_map<int,bool> visited;
            unordered_map<int,int> parent;
            queue<int> q;
            q.push(src);
            visited[src]=true;
            parent[src]=-1;
            while(!q.empty()){
                int frontNode = q.front();
                q.pop();
                for(auto nbr:adj[frontNode]){
                    if(visited[nbr]==false){
                        q.push(nbr);
                        visited[nbr]=true;
                        parent[nbr]=frontNode;
                    }
                }
            }
            vector<int> path;
            int node = destination;
            while (node!=-1){
                path.push_back(node);
                node = parent[node];

            }
            //reverse path
            reverse(path.begin(),path.end());
            cout<<"SHORTEST PATH"<<endl;
            for(auto i:path){
                cout<<i<<"->";
            }
            cout<<endl;
        }
        void topoSortDfs(int src,unordered_map<int,bool> &visited,stack<int> &ans){
            visited[src]=true;
            for(auto nbr:adj[src]){
                if(!visited[nbr]){
                    topoSortDfs(nbr,visited,ans);
                }
            }
            ans.push(src);
        }
        void shortestPathDirected(int src){
            stack<int> topoOrder;
            unordered_map<int,bool> visited;
            topoSortDfs(src,visited,topoOrder);
            int n = 0;
            for (auto it : adjWt) {
                n = max(n, it.first);
                for (auto p : it.second) {
                    n = max(n, p.first);
                }
            }
            vector<int> dist(n + 1, INT_MAX);

            src = topoOrder.top();
            topoOrder.pop();
            dist[src]=0;
            for(auto nbr:adjWt[src]){
                int node = nbr.first;
                int weightDistance = nbr.second;

                if(dist[src]+weightDistance<dist[node]){
                    dist[node]=dist[src]+weightDistance;
                }
            }
            //main logic
            while(!topoOrder.empty()){
                int frontNode = topoOrder.top();
                topoOrder.pop();
                for(auto nbr:adjWt[frontNode]){
                    int node = nbr.first;
                    int weightDistance = nbr.second;

                    if(dist[frontNode]+weightDistance<dist[node]){
                        dist[node]=dist[frontNode]+weightDistance;
                    }
                }
            }
            cout<<"Printing Distance array:"<<endl;
            for(auto i:dist){
                cout<<i<<",";
            }
            cout<<endl;
        }
};
int main(){
    Graph g;
    g.addEdge(0,1,0);
    g.addEdge(1,2,0);
    g.addEdge(2,3,0);
    g.addEdge(3,7,0);
    g.addEdge(3,4,0);
    g.addEdge(3,5,0);
    g.addEdge(4,6,0);
    g.addEdge(7,6,0);

    g.printAdjList(8);
    g.shortestDestination(0,7);

    cout<<"DIRECTED WEIGHTED GRAPH"<<endl;
    g.addEdgeWeight(0,1,5,1);
    g.addEdgeWeight(0,4,3,1);
    g.addEdgeWeight(0,2,12,1);
    g.addEdgeWeight(1,4,2,1);
    g.addEdgeWeight(1,2,7,1);
    g.addEdgeWeight(4,3,6,1);
    g.addEdgeWeight(3,2,2,1);
    
    g.shortestPathDirected(0);
    return 0;
}