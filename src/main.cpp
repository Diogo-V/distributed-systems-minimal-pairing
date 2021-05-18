#include <iostream>
#include <vector>
#include <deque>
#include <queue>

#define MAX 10000000

/* Represent processes X and Y. Y is going to be put in second last position of our adjacency array
 * and X is going to be put in the last position. X represents the source and Y the sink */
int X = 0, Y = 0;

using namespace std;

/**
 * @brief Represents a node's color. Used mainly during DFS.
 *
 * @param white node has not been visited
 * @param black node has been visited
 */
enum class Color { white, black};


/**
 * @brief Holds information about the related node.
 *
 * @param color represents the current state of the node
 * @param bfsParent holds the parent in the bfs
 * @param bfsParentPosition holds the position of this node if the adjacent's list of its parent
 */
typedef struct nodeInfoStruct {
    Color color;
    int bfsParent;
    int bfsParentPosition;
    nodeInfoStruct() {
        bfsParent = -1;
        color = Color::white;
        bfsParentPosition = -1;
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
    vector<vector<pair<int, pedgeInfoStruct>>> _adjacent;

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
        return this->getAdjacentNodes(parent)[child].second;
    };

    /**
     * @brief Get the Adjacent Nodes object.
     *
     * @param node node value
     * @return vector<pair<int, edgeInfoStruct>> list of connected nodes and their weights
     */
    vector<pair<int, pedgeInfoStruct>> getAdjacentNodes(int node) { return this->_adjacent[node]; };

    /**
     * @brief Get the Number of Nodes object.
     *
     * @return number of nodes
     */
    int getNumberOfNodes() const { return this->_numberOfNodes; };


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
        this->_adjacent[parent].push_back(make_pair(child, edge));
        this->_adjacent[child].push_back(make_pair(parent, edge));

    }

    /**
     * @brief Changes node color
     *
     * @param node node value
     * @param color node color
     */
    void setColor(int node, Color color) { this->_nodeInfo[node].color = color; }

    /**
     * @brief Changes node position
     * @param node node value
     * @param pos new position of node if the adjacent's list of its parent in bfs
     */
    void setPosition(int node, int pos) { this->_nodeInfo[node].bfsParentPosition = pos; }

    /**
     * @brief Change node parent
     * @param node node value
     * @param parent new parent of node in bfs
     */
    void setParent(int node, int parent) { this->_nodeInfo[node].bfsParent = parent; }

    /**
     * @brief Executes a bfs
     * @return boolean
     */
    bool bfs(){
        /* Mark all vertex as not visited*/
        for (int i = 0; i < this->getNumberOfNodes(); i++){
            this->setColor(i, Color::white);
        }
        /* Mark the source as visited */
        this->setColor(X, Color::black);

        queue<int> bfsAux;
        bfsAux.push(X);

        /* Proceeds with the BFS*/
        while (!bfsAux.empty()) {
            int otherNode;
            int i = 0;
            int node = bfsAux.front();
            bfsAux.pop();

            /* Iterates through nodes adjacent list */
            for (auto & it : this->getAdjacentNodes(node)) {
                otherNode = it.first;
                /* If the current node is the sink, we found a path*/
                if(otherNode == Y && it.second->residualCapacity() > 0){
                    this->setParent(Y, node);
                    this->setPosition(otherNode, i);
                    return true;
                }
                /* We will check if has a path in the Residual Graph and we haven't already visited the node*/
                if(this->getNodeInfo(otherNode).color == Color::white && it.second->residualCapacity() > 0){
                    /* The vertex will be marked as visited*/
                    this->setColor(otherNode, Color::black);
                    /* We will push the vertex in the bfsAux*/
                    bfsAux.push(otherNode);
                    this->setParent(otherNode, node);
                    this->setPosition(otherNode, i);
                }

                i++;
            }

            /* This vertex has been visited*/
            this->setColor(node, Color::black);
        }

        /* We were unable to find a path from X to Y*/
        return false;
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

/**
 * @brief Given a graph, finds the maximum flow with the edmonds karp, using a bfs to find the augmentations paths.
 *
 * @param graph
 */
void solveDistributedSystems(Graph* graph){
    /* Maximum flow initialized at 0 as every edge has 0 current flow*/
    int max_flow = 0;

    /* While there is an augmentation path from X to Y */
    while (graph->bfs()) {
        int minRC = MAX;
        int v;

        /* In the augmentation path we found, we find the minimum residual capacity*/
        for (v = Y; v != X; v = graph->getNodeInfo(v).bfsParent) {
            minRC = min(minRC, graph->getEdgeInfo(graph->getNodeInfo(v).bfsParent, graph->getNodeInfo(v).bfsParentPosition)->residualCapacity());
        }

        /* In the augmentation path we found, we increase the vertex's flow by the minimum residual capacity */
        for (v = Y; v != X; v = graph->getNodeInfo(v).bfsParent) {
            graph->getEdgeInfo(graph->getNodeInfo(v).bfsParent, graph->getNodeInfo(v).bfsParentPosition)->setCurrentFlux(minRC);
        }

        /*Add the minimum capacity to the maximum flow*/
        max_flow += minRC;
    }


    /* Outputs final result */
    cout << max_flow << endl;
}


/**
 * @brief Driver
 *
 * @return terminate code
 */
int main() {

    /* Initializes problem's structure and variables */
    Graph graph = initGraph();

    /* Finds maximum flow with the edmonds-karp algorithm. Prints it on the screen */
    solveDistributedSystems(&graph);

    exit(EXIT_SUCCESS);
}
