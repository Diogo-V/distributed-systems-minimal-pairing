#include <iostream>
#include <memory.h>
#include <vector>
#include <deque>
#include <queue>
#include <unordered_map>

#define MAX 10000000

/* Represent processes X and Y. Y is going to be put in second last position of our adjacency array
 * and X is going to be put in the last position. X represents the source and Y the sink */
int X = 0, Y = 0;


using namespace std;


/**
 * @brief Represents a node's color. Used mainly during DFS.
 *
 * @param white node has not been reached
 * @param grey node has been put in stack
 * @param black node has been totally visited
 */
enum class Color { white, black, grey };  // TODO: não sei se vai ser preciso mas deixei aqui para referencia


/**
 * @brief Holds information about the related node.
 *
 * @param color represents the current state of the node
 * @param bfsParent holds the parent in the bfs
 * @param dist holds distance to another Source
 */
typedef struct nodeInfoStruct {  // TODO: Não sei se vai ser preciso, mas deixei aqui para referência
    Color color;
    int bfsParent;
    int dist;
    nodeInfoStruct() {
        bfsParent = -1;
        dist = -1;
    };
} nodeInfoStruct;


/**
 * @brief Hold info related to the edge.
 *
 * @param current_flux flux that is currently being passed
 * @param max_flux maximum flux that can be passed in this edge
 */
typedef struct edgeInfoStruct {
    int current_flux;
    int max_flux;

    explicit edgeInfoStruct(int weight) {
        current_flux = 0;
        max_flux = weight;
    }

    int residualCapacity(){
        return max_flux - current_flux;
    }

    //TODO: alterar nome
    void setCurrentFlux(int n){
        this->current_flux += n;
    }

} * pedgeInfoStruct;


/**
 * @brief Represents a Weighted Un-Directed Graph. Uses adjacency list.
 */
class Graph {

private:

    /**
     * @brief Holds all the info related to the key node during the algorithm.
     */
    vector<nodeInfoStruct> _nodeInfo;

    /**
     * @brief Holds all the nodes which this node leads to.
     */
    vector<unordered_map<int, pedgeInfoStruct>> _adjacent;

    /**
     * @brief Holds number of vertices inside this graph.
     */
    int _numberOfNodes;


public:

    /**
     * @brief Graph constructor.
     *
     * @param nodes number of nodes inside graph
     */
    explicit Graph(int nodes) {

        /* Saves number of nodes. We sum 2 because of X and Y nodes */
        this->_numberOfNodes = nodes + 2;

        /* Populates info vector with all the possible nodes */
        nodeInfoStruct info;
        this->_nodeInfo.resize(this->getNumberOfNodes(), info);

        /* Creates and allocates space for all the nodes' connections */
        this->_adjacent.resize(this->getNumberOfNodes());

    };

    /**
     * @brief Get the Node Info object.
     *
     * @param node node value
     * @return nodeInfoStruct related to this node
     */
    nodeInfoStruct getNodeInfo(int node) { return this->_nodeInfo[node]; };

    /**
     * @brief Get the Edge Info object.
     *
     * @param parent parent node
     * @param child child node
     * @return pair<int, pedgeInfoStruct> edge info between this two nodes
     */
    pedgeInfoStruct getEdgeInfo(int parent, int child) {
        return this->getAdjacentNodes(parent)[child];
    };

    /**
     * @brief Get the Adjacent Nodes object.
     *
     * @param node node value
     * @return vector<pair<int, edgeInfoStruct>> list of connected nodes and their weights
     */
    unordered_map<int, pedgeInfoStruct> getAdjacentNodes(int node) { return this->_adjacent[node]; };

    /**
     * @brief Get the Number of Nodes object.
     *
     * @return number of nodes
     */
    int getNumberOfNodes() const { return this->_numberOfNodes; };


    /**
     * @brief Changes node's distance.
     *
     * @param node node to be changed
     * @param dist new distance
     */
    // TODO: Deixei esta aqui para teres uma referência de como modificar a informação de um nó
    void setNodeDistance(int node, int dist) { this->_nodeInfo[node].dist = dist; };

    /**
     * @brief Inserts a new bi-directional edge between two nodes.
     *
     * @param parent parent's node
     * @param child child's node
     * @param weight edge's weight
     */
    void addEdge(int parent, int child, int weight) {

        /* Creates a connection between two nodes */
        auto edge = new edgeInfoStruct(weight);
        this->_adjacent[parent].insert(make_pair(child, edge));
        this->_adjacent[child].insert(make_pair(parent, edge));

    }

    /**
     * @brief Performs an iterative DFS traversal of this graph starting from node 1.
     *
     * @return deque with nodes in the minimum path from X to Y
     */
    deque<int> dfs() {  

        /* Holds nodes that we are visiting or are about to visit */
        deque<int> dfsAux;

        /* Holds nodes that have been already visited in topological order */
        deque<int> topological;

        /* Visits each node (domino piece) in our graph */
        for (int parent = 1; parent <= this->getNumberOfNodes(); parent++) {

            /* If it has not been yet visited, we put it inside our dfs aux */
            if (this->getNodeInfo(parent).color == Color::white)
                dfsAux.push_front(parent);

            /* We keep visiting node until all the nodes have turned black (fully visited) */
            while (!dfsAux.empty()) {

                /* Gets last node being put inside our auxiliary. It mimics what a recursion
                 * would have done */
                int node = dfsAux.front();

                /* If we have already visited everything from this node, we put it in our topological
                 * stack and go to next iteration */
                if (this->getNodeInfo(node).color == Color::black) {
                    dfsAux.pop_front();
                    topological.push_front(node);
                    continue;
                }

                /* Since we are about to visit each child from this node, we set it as grey */
                // this->setNodeColor(node, Color::grey);

                /* Puts every not yet visited child node inside aux */
//                for (int son : this->getAdjacentNodes(node))
//                    if(this->getNodeInfo(son).color == Color::white)
//                        dfsAux.push_front(son);

                /* Since we have put all it's children inside the aux, it has finished */
                // this->setNodeColor(node, Color::black);

            }

        }

        return topological;
    }


    bool bfs(int parent[], int position[]){
        // Create a visited array and mark all vertices as not
        // visited
        // TODO: utilizar cores
        bool visited[getNumberOfNodes()];
        memset(visited, 0, sizeof(visited));

        queue<int> bfsAux;
        bfsAux.push(X);
        visited[X] = true;
        parent[X] = -1;  /* Saves node's parent */
        position[X] = -1;  /*  */
        // TODO: colocar como parâmetros para os nós

        /* Proceeds with the BFS*/
        while (!bfsAux.empty()) {
            int otherNode;
            int node = bfsAux.front();
            int i = 0;
            bfsAux.pop();

            for (auto & it : this->getAdjacentNodes(node)) {
                otherNode = it.first;
                //printf("%d\n", otherNode);
                /* If the current node is the sink, we found an path*/
                if(otherNode == Y && it.second->residualCapacity() > 0){
                    parent[Y] = node;
                    position[otherNode] = i;
                    return true;
                }
                /* We will check if has a path in the Residual Graph and we haven't already visited the node*/
                if(!visited[otherNode] && it.second->residualCapacity() > 0){
                    /* The vertex will be marked as visited*/
                    visited[otherNode] = true;
                    /* We will push the vertex in the bfsAux*/
                    bfsAux.push(otherNode);
                    parent[otherNode] = node;
                    position[otherNode] = i;
                }

                i++;
            }

            visited[node] = true;
        }

        /* We were unable to find a path from X to Y */
        return false;
    }

    /**
     * @brief Used to print graph's connections. Mainly used for debugging purposes.
     */
    void print() {
        int v, w;
        for (int u = 0; u < this->getNumberOfNodes(); u++) {
            cout << "Node " << u + 1 << " makes an edge with \n";
            for (auto & it : this->getAdjacentNodes(u)) {
                v = it.first;
                w = it.second->max_flux;
                cout << "\tNode " << v + 1 << " with edge weight = " << w << "\n";
            }
            cout << "\n";
        }
    }
};


/**
 * @brief Inits values for X and Y after reading number of nodes from cin.
 *
 * @param nNodes number of nodes inside graph
 */
void initXAndY(int nNodes) {
    X = nNodes;
    Y = nNodes + 1;
}


/**
 * @brief Creates and populates the graph that is going to represent our problem.
 *
 * @return newly created graph
 */
Graph initGraph() {

    /* Receive number of nodes and number of edges between processes (communications) from cin */
    int nNodes = 0, nCom = 0;

    /* Holds edge's cost while populating */
    int costX = 0, costY = 0, costCom = 0;

    /* Holds processes' nodes while populating communications */
    int from = 0, to = 0;

    /* Reads number of nodes and edges from cin */
    scanf("%d %d", &nNodes, &nCom);

    /* Creates graph and init X and Y values */
    Graph graph(nNodes); initXAndY(nNodes);

    /* Populates connections between processors and processes */
    for (int node = 0; node < nNodes; node++) {
        scanf("%d %d", &costX, &costY);
        graph.addEdge(node, X, costX);
        graph.addEdge(node, Y, costY);
    }

    /* Populates connections between processes. We subtract 1 to remove the offset */
    for (int i = 0; i < nCom; i++) {
        scanf("%d %d %d", &from, &to, &costCom);
        graph.addEdge(from - 1, to - 1, costCom);
    }

    return graph;

}


void solveDistributedSystems(Graph* graph){

    /* Array of parents, populated by the BFS*/
    int parent[graph->getNumberOfNodes()];

    /* Array of positions in parent's list of adjacencies, populated by the BFS*/
    int position[graph->getNumberOfNodes()];

    int max_flow = 0;

    /* While there is an augmentation path from X to Y */
    while (graph->bfs(parent, position)) {
        /* Find the augmentation path and increase the vertex's flow by the minimum residual capacity */
        int minRC = MAX;
        int v;

        for (v = Y; v != X; v = parent[v]) {
            minRC = min(minRC, graph->getEdgeInfo(parent[v], position[v])->residualCapacity());
        }


        for (v = Y; v != X; v = parent[v]) {
            graph->getEdgeInfo(parent[v], position[v])->setCurrentFlux(minRC);
            /*u = parent[v];

            //TODO: posso melhorar

            for (auto & it : graph->getAdjacentNodes(v)){
                if(it.first == u) {
                    it.second->setCurrentFlux(minRC);
                    break;
                }
            }*/

        }

        // Add path flow to overall flow
        max_flow += minRC;


        /* Resets parents array*/
        //for (v = 0; v < graph->getNumberOfNodes(); v++){
          //  parent[v] = -1;
        //}
    }


    // Return the overall flow //TODO: precisamos de alterar
    cout << max_flow << endl;
}


/**
 * @brief Driver code.
 *
 * @return terminate code
 */
int main() {

    /* Initializes problem's structure and variables */
    Graph graph = initGraph();

    /* TODO: Remove this. Used for debug */
    // graph.print();

    solveDistributedSystems(&graph);

    exit(EXIT_SUCCESS);
}
