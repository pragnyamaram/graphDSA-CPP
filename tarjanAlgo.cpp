#include<iostream>
#include<unordered_map>
#include<list>
#include<vector>
using namespace std;

class graph{
    unordered_map<int,list<int>> adj;
    public:
        void addEdge(int u,int v,bool direction){
            if(direction==1){
                adj[u].push_back(v);
            }
            else{
                adj[u].push_back(v);
                adj[v].push_back(u);
            }
        }
        void printAdjList(int n){
            for(int i=0;i<n;i++){
                cout<<i<<"-> {";
                for(auto nbr:adj[i]){
                    cout<<nbr<<",";
                }
                cout<<"}"<<endl;
            }
        }
         void countBridge(int src,int parent,vector<int>&tin,vector<int>&low,unordered_map<int,bool> &vis,int &timer){
            timer++;
            vis[src]=true;
            tin[src]=timer;
            low[src]=timer;
            //neighbors pe travel
            for(auto nbr:adj[src]){
                //undirected graph may include parent too
                if(nbr==parent){
                    continue;
                }
                else if(vis[nbr]==false){
                    //normal traversal karo and check for bridges
                    countBridge(nbr,src,tin,low,vis,timer);
                    low[src]=min(low[src],low[nbr]);
                    //ad mein wapas agayu hu bridge check
                    if(low[nbr]>tin[src]){
                        //bridge exists
                        cout<<"bridge exists"<<endl;
                        cout<<nbr<<"-"<<src<<endl;
                    }
                }
                else{
                    //already visited and not parent
                    //low time ko update kardo
                    low[src]=min(low[src],low[nbr]);
                }
            }
        }   
};

int main(){
    graph g;
    g.addEdge(0,1,0);
    g.addEdge(0,2,0);
    g.addEdge(2,3,0);
    g.addEdge(1,2,0);
    g.addEdge(3,4,0);
    int n = 5;
    g.printAdjList(n);
    int timer = 0;
    vector<int> tin(n,0);
    vector<int> low(n,0);
    unordered_map<int,bool> vis;
    g.countBridge(0,-1,tin,low,vis,timer);

}