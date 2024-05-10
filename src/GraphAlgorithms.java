import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Queue;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.PriorityQueue;

/**
 * Your implementation of various different graph algorithms.
 *
 * @author Dean Miyata-Dawson
 * @userid ddawson42
 * @GTID 903833148
 * @version 1.0
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * You may import/use java.util.Set, java.util.List, java.util.Queue, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("Error: start is null.");
        }
        if (graph == null) {
            throw new IllegalArgumentException("Error: graph is null.");
        }

        List<Vertex<T>> list = new ArrayList<>();
        Set<Vertex<T>> visited = new HashSet<>();
        Queue<Vertex<T>> q = new LinkedList<>();
        visited.add(start);
        q.add(start);

        while (!(q.isEmpty())) {
            Vertex<T> v = q.remove();
            list.add(v);
            for (VertexDistance<T> vertex: graph.getAdjList().get(v)) {
                if (!(visited.contains(vertex.getVertex()))) {
                    q.add(vertex.getVertex());
                    visited.add(vertex.getVertex());
                }
            }
        }

        return list;
    }

    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * all points for this method.
     *
     * You may import/use java.util.Set, java.util.List, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("Error: start is null.");
        }
        if (graph == null) {
            throw new IllegalArgumentException("Error: graph is null.");
        }

        List<Vertex<T>> list = new ArrayList<>();
        Set<Vertex<T>> visited = new HashSet<>();

        dfsHelper(start, graph, visited, list);

        return list;
    }

    /**
     * Helper method for Depth-First Search
     * @param vertex current vertex node in graph
     * @param graph graph we are searching
     * @param visited set of all visited vertices in graph
     * @param list of vertices in order
     * @param <T> generic type of data in vertex
     */
    private static <T> void dfsHelper(Vertex<T> vertex, Graph<T> graph, Set<Vertex<T>> visited, List<Vertex<T>> list) {
        list.add(vertex);
        visited.add(vertex);

        for (VertexDistance<T> v : graph.getAdjList().get(vertex)) {
            if (!(visited.contains(v.getVertex()))) {
                dfsHelper(v.getVertex(), graph, visited, list);
            }
        }
    }


    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing
     * infinity) if no path exists.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Map, and java.util.Set and any class that
     * implements the aforementioned interfaces, as long as your use of it
     * is efficient as possible.
     *
     * You should implement the version of Dijkstra's where you use two
     * termination conditions in conjunction.
     *
     * 1) Check if all of the vertices have been visited.
     * 2) Check if the PQ is empty.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the Dijkstra's on (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every
     * other node in the graph
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start,
                                                        Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("Error: start is null.");
        }
        if (graph == null) {
            throw new IllegalArgumentException("Error: graph is null.");
        }

        Map<Vertex<T>, Integer> map = new HashMap<>();
        Set<Vertex<T>> visited = new HashSet<>();
        Queue<VertexDistance<T>> pq = new PriorityQueue<>();

        for (Vertex<T> v : graph.getAdjList().keySet()) {
            if (v.equals(start)) {
                map.put(v, 0);
            } else {
                map.put(v, Integer.MAX_VALUE);
            }
        }

        pq.add(new VertexDistance<>(start, 0));

        while (!(pq.isEmpty())) {
            VertexDistance<T> removed = pq.remove();

            for (VertexDistance<T> vertex : graph.getAdjList().get(removed.getVertex())) {
                if (map.get(vertex.getVertex()) > removed.getDistance() + vertex.getDistance()) {
                    map.put(vertex.getVertex(), removed.getDistance() + vertex.getDistance());
                    pq.add(new VertexDistance<T>(vertex.getVertex(), removed.getDistance() + vertex.getDistance()));
                }
            }
        }

        return map;
    }

    /**
     * Runs Prim's algorithm on the given graph and returns the Minimum
     * Spanning Tree (MST) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid MST exists, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * reverse edge to the set as well. This is for testing purposes. This
     * reverse edge does not need to be the one from the graph itself; you can
     * just make a new edge object representing the reverse edge.
     *
     * You may assume that there will only be one valid MST that can be formed.
     *
     * You should NOT allow self-loops or parallel edges in the MST.
     *
     * You may import/use PriorityQueue, java.util.Set, and any class that 
     * implements the aforementioned interface.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for this method (storing the adjacency list in a variable is fine).
     *
     * @param <T> the generic typing of the data
     * @param start the vertex to begin Prims on
     * @param graph the graph we are applying Prims to
     * @return the MST of the graph or null if there is no valid MST
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Set<Edge<T>> prims(Vertex<T> start, Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("Error: start is null.");
        }
        if (graph == null) {
            throw new IllegalArgumentException("Error: graph is null.");
        }
        if (!(graph.getVertices().contains(start))) {
            throw new IllegalArgumentException("Error: start is not in graph.");
        }

        Set<Vertex<T>> visited = new HashSet<>();
        Set<Edge<T>> edges = new HashSet<>();
        PriorityQueue<Edge<T>> pq = new PriorityQueue<>();

        for (VertexDistance<T> vertexPair : graph.getAdjList().get(start)) {
            pq.add(new Edge<T>(start, vertexPair.getVertex(), vertexPair.getDistance()));
        }

        visited.add(start);

        while (!(pq.isEmpty()) && !(visited.containsAll(graph.getVertices()))) {
            Edge<T> edge = pq.poll();

            if (!(visited.contains(edge.getV()))) {
                visited.add(edge.getV());
                edges.add(edge);
                edges.add(new Edge<T>(edge.getV(), edge.getU(), edge.getWeight()));

                for (VertexDistance<T> vp : graph.getAdjList().get(edge.getV())) {
                    if (!visited.contains(vp.getVertex())) {
                        pq.add(new Edge<T>(edge.getV(), vp.getVertex(), vp.getDistance()));
                    }
                }
            }
        }
        if (edges.size() != 2 * (graph.getVertices().size() - 1)) {
            return null;
        }
        return edges;
    }
}
