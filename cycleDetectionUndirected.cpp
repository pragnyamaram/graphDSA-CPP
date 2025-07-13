#include<iostream>
#include<unordered_map>
#include<list>
using namespace std;
class Graph{
    public:
    unordered_map<int,list<int>> adjListUndirected;
    unordered_map<int,list<int>> adjListDirected;
    void addEdgeUndirected(int u,int v){
        adjListUndirected[u].push_back(v);
        adjListUndirected[v].push_back(u);
    }
    void addEdgeDirected(int u,int v){
        adjListDirected[u].push_back(v);
    }
    void printGraphUndirected(int n){
        for(int i=0;i<n;i++){
            cout<<i<<":"<<"{";
            list<int> temp = adjListUndirected[i];
            for(auto j:temp){
                cout<<j<<",";
            }
            cout<<"}"<<endl;
        }
    }
    void printGraphDirected(int n){
        for(int i=0;i<n;i++){
            cout<<i<<":"<<"{";
            list<int> temp = adjListDirected[i];
            for(auto j:temp){
                cout<<j<<",";
            }
            cout<<"}"<<endl;
        }
    }

    
};
int main(){
    Graph g;
    //undirected Graph
    g.addEdgeUndirected(1,2);
    g.addEdgeUndirected(2,4);
    g.addEdgeUndirected(2,3);
    g.addEdgeUndirected(4,5);
    g.addEdgeUndirected(3,5);
    cout<<"UNDIRECTED GRAFH"<<endl;
    g.printGraphUndirected(6);

    g.addEdgeDirected(1,2);
    g.addEdgeDirected(1,3);
    g.addEdgeDirected(2,3);
    g.addEdgeDirected(2,4);
    g.addEdgeDirected(5,4);
    g.addEdgeDirected(5,3);
    cout<<"DIRECTED GRAFH"<<endl;
    g.printGraphDirected(6);



}