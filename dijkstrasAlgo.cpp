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
        void shortestDistanceDijkstraAlgo(int src,int n){
            vector<int> distance(n+1,INT_MAX);
            set<pair<int,int>> st;
            distance[src]=0;
            st.insert({0,src});
            while(!st.empty()){
                auto top = *(st.begin());
                int topNodeValue = top.first;
                int topNode = top.second;
                //pop top
                st.erase(st.begin());
                for(auto nbr:adjWt[topNode]){
                    //nbr is a pair
                    //nbr->{a,b} a->node b->wt
                    if(topNodeValue+nbr.second < distance[nbr.first]){
                        //distance array ko update
                        //set ko update karna hoga with that particular value
                        auto result = st.find({nbr.second,nbr.first});
                        if(result!=st.end()){
                            st.erase(result);
                        }        
                        distance[nbr.first] = topNodeValue + nbr.second;
                        st.insert({distance[nbr.first],nbr.first});
                    }
                }
            }
            cout<<"Printing the result:"<<endl;
            for(int i=0;i<n;i++){
                cout<<distance[i]<<",";
            }
        }

};
int main(){
    graph g;
    g.addEdgeWeight(0,3,6,0);
    g.addEdgeWeight(0,5,9,0);
    g.addEdgeWeight(5,1,14,0);
    g.addEdgeWeight(5,4,2,0);
    g.addEdgeWeight(4,3,11,0);
    g.addEdgeWeight(4,1,9,0);
    g.addEdgeWeight(4,2,10,0);
    g.addEdgeWeight(1,2,7,0);
    g.addEdgeWeight(2,3,15,0);
    g.printAdjList(6);
    g.shortestDistanceDijkstraAlgo(0,6);

}