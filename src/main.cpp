#include <iostream>
#include <vector>
#include <deque>
#include <unordered_map>


/* Represent processes X and Y. X is going to be put in second last position of our adjacency array
 * and Y is going to be put in the last position */
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
 * @param inDegree holds the amount of nodes connected to this node
 * @param dist holds distance to another Source
 */
typedef struct nodeInfoStruct {  // TODO: Não sei se vai ser preciso, mas deixei aqui para referência
    Color color;
    int inDegree;
    int dist;
    nodeInfoStruct() {
        color = Color::white;
        inDegree = 0;
        dist = 1;
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
} edgeInfoStruct;


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
    vector<unordered_map<int, edgeInfoStruct>> _adjacent;

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
     * @return pair<int, edgeInfoStruct> edge info between this two nodes
     */
    edgeInfoStruct getEdgeInfo(int parent, int child) {
        return this->getAdjacentNodes(parent).find(child)->second;
    };

    /**
     * @brief Get the Adjacent Nodes object.
     *
     * @param node node value
     * @return unordered_map<int, edgeInfoStruct> map of connected nodes and their weights
     */
    unordered_map<int, edgeInfoStruct> getAdjacentNodes(int node) { return this->_adjacent[node]; };

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
        edgeInfoStruct edge(weight);
        this->_adjacent[parent].insert(make_pair(child, edge));
        this->_adjacent[child].insert(make_pair(parent, edge));

    }

    /**
     * @brief Performs an iterative DFS traversal of this graph starting from first node (1).
     *
     * @return deque with nodes in topological order
     */
    deque<int> dfs() {  // TODO: Deixei a DFS para o caso de quereres tirar alguma ideia

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

    /**
     * @brief Used to print graph's connections. Mainly used for debugging purposes.
     */
    void print() {
        int v, w;
        for (int u = 0; u < this->getNumberOfNodes(); u++) {
            cout << "Node " << u << " makes an edge with \n";
            for (auto & it : this->getAdjacentNodes(u)) {
                v = it.first;
                w = it.second.max_flux;
                cout << "\tNode " << v << " with edge weight =" << w << "\n";
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


/**
 * @brief Driver code.
 *
 * @return terminate code
 */
int main() {

    /* Initializes problem's structure and variables */
    Graph graph = initGraph();

    /* TODO: Remove this. Used for debug */
    graph.print();

    return 0;
}
