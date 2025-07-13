#include<iostream>
#include<list>
#include<queue>
#include<unordered_map>
#include<vector>
#include<stack>
using namespace std;

class Graph{
    public:
        unordered_map<int,list<int>> adjWt;

        void addEdge(int u,int v, bool direction){
            if(direction==1){
                adjWt[u].push_back(v);
            }
            else{
                adjWt[u].push_back(v);
                adjWt[v].push_back(u);
            }
        }
        void printAdjList(int n){
            for(int i=0;i<n;i++){
                cout<<i<<"-> {";
                for(auto nbr:adjWt[i]){
                    cout<<nbr<<",";
                }
                cout<<"}"<<endl;
            }
        }
        void topoSortDFS(stack<int>& st,int node,unordered_map<int,bool>& vis){
            vis[node]=true;
            for(auto nbr:adjWt[node]){
                if(!vis[nbr]){
                    //vis[node]=true;
                    topoSortDFS(st,nbr,vis);
                }
            }
            st.push(node);
        }
        void dfs(int src,unordered_map<int,bool>&visited,unordered_map<int,list<int>>adjNew){
            visited[src]=true;
            cout<<src<<"-";
            for(auto nbr:adjNew[src]){
                if(!visited[nbr]){
                    dfs(nbr,visited,adjNew);
                }
            }
        }
        int countSCC(int n){
            //find topo ordering 
            //step1:topological sort
            stack<int> st;
            unordered_map<int,bool> vis;
            for(int i=0;i<n;i++){
                if(!vis[i]){
                    topoSortDFS(st,i,vis);
                }
            }
            //yaha par topo is ordering ready hea

            //step 2:all connecting edges ko rverse karna hoga
            unordered_map<int,list<int>> adjNew;
            for(auto i:adjWt){
                for(auto j:i.second){
                    int u = i.first;
                    int v = j;
                    //u->v ek edge thi
                    //reverse v->u
                    adjNew[v].push_back(u);
                }
            }
            //traversal 
            int count = 0;
            unordered_map<int,bool>visited;
            while(!st.empty()){
                int src = st.top();
                st.pop();
                if(!visited[src]){
                    cout<<"SCC:";
                    dfs(src,visited,adjNew);
                    //ek pura component traverse ho chuka hea
                    cout<<endl;
                    count++;
                }
            }
            return count;
        }
};
int main(){
    Graph g;
    g.addEdge(1,0,1);
    g.addEdge(0,3,1);
    g.addEdge(3,2,1);
    g.addEdge(2,1,1);
    g.addEdge(2,4,1);
    g.addEdge(4,5,1);
    g.addEdge(5,6,1);
    g.addEdge(6,4,1);
    g.addEdge(6,7,1);
    int n = 7;
    g.printAdjList(n);
    
    int sccCount = g.countSCC(n);
    cout<<"SCC COUNT:"<<sccCount;

    return 0;
}