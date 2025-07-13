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
        void sortestDistanceBelamnFord(int src,int n){
            vector<int> distance(n,INT_MAX);
            distance[src]=0;
            for(int i=0;i<n-1;i++){
                //hr edge pr jao relaxation step karo
                for(auto a:adjWt){
                    for(auto b:a.second){
                        int u = a.first;
                        int v = b.first;
                        int wt = b.second;
                        if(distance[u]!=INT_MAX && distance[u]+wt<distance[v]){
                            distance[v]=distance[u]+wt;

                        }
                    }
                }
            }
            bool negativeCycleFlag = false;
            for(auto a:adjWt){
                for(auto b:a.second){
                    int u = a.first;
                    int v = b.first;
                    int wt = b.second;
                    if(distance[u]!=INT_MAX && distance[u]+wt<distance[v]){
                        distance[v]=distance[u]+wt;
                        negativeCycleFlag = true;
                        break;
                    }
                }
            }
            cout<<"Printing bellman result:"<<endl;
            for(int i=0;i<n;i++){
                cout<<distance[i]<<",";
            }
            cout<<endl;
            if(negativeCycleFlag==true){
                cout<<"negitive cycle is present"<<endl;
            }
            else{
                cout<<"negative cycle is not present"<<endl;
            }
        }
};
int main(){
    graph g;
    g.addEdgeWeight(0,1,-1,1);
    g.addEdgeWeight(1,4,2,1);
    g.addEdgeWeight(4,3,-3,1);
    g.addEdgeWeight(3,2,5,1);
    g.addEdgeWeight(2,0,4,1);
    g.addEdgeWeight(1,2,3,1);
    g.addEdgeWeight(1,3,2,1);
    g.addEdgeWeight(3,2,5,1);

    g.printAdjList(5);
    g.sortestDistanceBelamnFord(0,5);
    
}