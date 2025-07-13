#include<iostream>
#include<unordered_map>
#include<list>
#include<set>
#include<vector>
#include<limits.h>
using namespace std;

class graph{
    public:
        unordered_map<int,list<pair<int,int>>> adjWt;

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
                cout<<"{";
                list<pair<int,int>> temp = adjWt[i];
                for(auto j:temp){
                    cout<<"("<<j.first<<","<<j.second<<"),";
                }
                cout<<"}"<<endl;
            }
        
        }
        void floyWarshalAlgo(int n){
            vector<vector<int>> dist(n,vector<int> (n,INT_MAX/2));
            //step 1: diagonal pe 0 daldho
            for(int i=0;i<n;i++){
                dist[i][i]=0;
            }
            //step 2: copy all the distances of graph to matrix
            for(auto a:adjWt){
                for(auto b:a.second){
                    int u = a.first;
                    int v = b.first;
                    int wt = b.second;
                    dist[u][v]=wt;
                }
            }
            //step 3 : main logic
            for(int helper = 0;helper<n;helper++){
                for(int src = 0;src<n;src++){
                    for(int dest=0;dest<n;dest++){
                        dist[src][dest] = min(dist[src][dest],dist[src][helper]+dist[helper][dest]); 
                    }
                }
            }
            cout<<"Printing result:"<<endl;
            for(int i=0;i<n;i++){
                for(int j=0;j<n;j++){
                    cout<<dist[i][j]<<",";
                }
                cout<<endl;
            }
        }

};
int main(){
    graph g;
    g.addEdgeWeight(0,2,-2,1);
    g.addEdgeWeight(1,0,4,1);   
    g.addEdgeWeight(1,2,3,1);
    g.addEdgeWeight(3,1,-1,1);
    g.addEdgeWeight(2,3,2,1);

    g.floyWarshalAlgo(4);
}