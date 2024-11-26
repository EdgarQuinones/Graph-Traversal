package p5;

import graph.Edge;
import graph.Graph;

import java.io.File;
import java.io.FileNotFoundException;
import java.lang.reflect.Array;
import java.util.*;

public class UndirectedGraph {

	Graph G; // Initialize this from within the constructor; the graph must be undirected

	public UndirectedGraph(String inputFile) throws FileNotFoundException {

		Scanner scnr = new Scanner(new File(inputFile));
		int vertexCount = Integer.parseInt(scnr.nextLine().trim());
		int edgeCount = Integer.parseInt(scnr.nextLine().trim());

		G = new Graph(vertexCount, false);

		for (int i = 0; i < edgeCount; i++) {
			String[] line = scnr.nextLine().split(" ");
			Edge edge = new Edge(Integer.parseInt(line[0]), Integer.parseInt(line[1]), Integer.parseInt(line[2]));
			G.addEdge(edge);
		}

	}

	public String toString() {

		// for loop for arraylist Adjacency map (i) using edgeIterator
		// for loop for number of links inside each index                // Get edge based on source[i] and destination
//        Iterator<Edge> edgeIterator = G.edgeIterator(0);
//
//        while (edgeIterator.hasNext()) {
//            Edge e = edgeIterator.next();
//            System.out.println(e.getWeight());
//
//        }
//        System.out.println(G.getEdge(0,99));
//
//        System.out.println(G.getEdge(0,1));

		StringBuilder graph = new StringBuilder();

		for (int i = 0; i < G.getVertexCount(); i++) {
			Iterator<Edge> edgeIterator = G.edgeIterator(i);
			graph.append(i).append(": ");

			while (edgeIterator.hasNext()) {
				Edge e = edgeIterator.next();
				graph.append("<").append(e.getDestination()).append(",").append((int)e.getWeight()).append("> ");
			}
			graph.append("\n");
		}


		return graph.toString();
	}

	public int countTriplets() {
		ArrayList<Edge> matchedEdges = new ArrayList<>();
		int threes = 0;

		for (int i = 0; i < G.getVertexCount(); i++) {

			Iterator<Edge> edgeIterator = G.edgeIterator(i);
			while(edgeIterator.hasNext()) {

				Edge edge1 = edgeIterator.next();
				Iterator<Edge> edgeIterator2 = G.edgeIterator(i);
				boolean found = false;

				while(edgeIterator2.hasNext()) {

					Edge edge2 = edgeIterator2.next();
					Edge edge3 = G.getEdge((int)edge1.getDestination(), (int)edge2.getDestination());

					boolean found1 = false;
					boolean found2 = false;
					boolean found3 = false;

					if (edge3 != null) {

						for (Edge e : matchedEdges) {
							if (edge1.equals(e) || Objects.equals(edge1.getDestination(), e.getSource()) && Objects.equals(edge1.getSource(), e.getDestination())) found1 = true;
							if (edge2.equals(e) || Objects.equals(edge2.getDestination(), e.getSource()) && Objects.equals(edge2.getSource(), e.getDestination())) found2 = true;
							if (edge3.equals(e) || Objects.equals(edge3.getDestination(), e.getSource()) && Objects.equals(edge3.getSource(), e.getDestination())) found3 = true;
						}


						if(!found1 || !found2 || !found3) {
							threes++;

							matchedEdges.add(edge1);
							matchedEdges.add(edge2);
							matchedEdges.add(edge3);

						}
						//check if all 3 edges arent all in the arrylist
							//if true, continue,
							//if false, add to list

					}
				}
			}

		}
		return threes;
	}

	public class Node {
		int vertex;
		ArrayList<Edge> edges = new ArrayList<>();

		public Node(int vertex) {
			this.vertex = vertex;

			Iterator<Edge> edgeIterator = G.edgeIterator(vertex);

			while(edgeIterator.hasNext()) {
				Edge e = edgeIterator.next();
				edges.add(e);
			}
		}
	}

	public int findShortestPathLengthBetween(int start, int end) {

		// shortest path from node START to node END
        int[] dValues = new int[G.getVertexCount()]; // array storing all d-values for ALL nodes

		// fill all nodes with infinite except the START node
		for (int i = 0; i < G.getVertexCount(); i++) {
			dValues[i] = Integer.MAX_VALUE;
		}
		dValues[start] = 0;

		ArrayList<Node> C = new ArrayList<>();
		for (int i = 0; i < G.getVertexCount(); i++) {
			C.add(new Node(i));
		}

		while (!C.isEmpty()) {

			// Find the node in C, that has the smallest d-value
			int smallestDValue = Integer.MAX_VALUE;
			int smallestIndex = 0;
			Node smallestNode = C.get(smallestIndex);

            for (int i = 0; i < C.size(); i++) {
				int otherNodeDValue = dValues[C.get(i).vertex];
				int smallestNodeDValue = dValues[smallestNode.vertex];

                if (smallestNodeDValue > otherNodeDValue) {
					smallestNode = C.get(i);
					smallestIndex = i;
                }
            }
			C.remove(smallestIndex);

			// Loops through every neighbor of the vertex
			for (int i = 0; i < smallestNode.edges.size(); i++) {
				Node neighbor = new Node(smallestNode.edges.get(i).getDestination());

				// The neighbor must still be in C
				boolean stillInC = false;
				for (Node node : C) {
					if (neighbor.vertex == node.vertex) {
						stillInC = true;
						break;
					}
				}

				// If the neighbor still is in C, compares D values
				if (stillInC) {
					int neighborDValue = dValues[neighbor.vertex];
					int smallestNodeDValuePLUSWeight = dValues[smallestNode.vertex] + (int)G.getEdge(smallestNode.vertex, neighbor.vertex).getWeight();

					if(neighborDValue > smallestNodeDValuePLUSWeight) {
						dValues[neighbor.vertex] = smallestNodeDValuePLUSWeight;
					}
				}

			}

		}

		return dValues[end];
}


	public String ifConnectedThenBreadthFirstTraversal() {
		StringBuilder path = new StringBuilder();
		int counts = 0;
		boolean[] discovered = new boolean[G.getVertexCount()];
		Arrays.fill(discovered, false);
		discovered[0] = true;

		Queue<Node> Q = new LinkedList<>();
		Q.offer(new Node(G.edgeIterator().next().getSource()));

		while (!Q.isEmpty()) {
			Node currentNode = Q.poll();
			path.append(currentNode.vertex).append(" ");
			counts++;

			for (int i = 0; i < currentNode.edges.size(); i++) {
				Node neighbor = new Node(currentNode.edges.get(i).getDestination());
				if(!discovered[neighbor.vertex]) {
					discovered[neighbor.vertex] = true;
					Q.offer(neighbor);
				}
			}

		}
		if(counts == G.getVertexCount()) return path.toString();
		else return null;

//		StringBuilder vertices = new StringBuilder();
//		Queue<Integer> queue = new LinkedList<>();
//		int verticesTracked = 0;
//		boolean[] visited = new boolean[G.getVertexCount()];
//		int currentVertex = 0;
//		visited[currentVertex] = true;
//		for (int i = 1; i < visited.length; i++) {
//			visited[i] = false;
//		}
//		queue.offer(currentVertex);
//		int counter = 0;
//		while (!queue.isEmpty()) {
//			if (counter > G.getVertexCount()) {
//				return null;
//			}
//			currentVertex = queue.poll();
//			Iterator<Edge> iterator = G.edgeIterator(currentVertex);
//			int numOfEdges = G.degree(currentVertex);
//			vertices.append(currentVertex).append(" ");
//			verticesTracked++;
//
//			while (iterator.hasNext()){
//				Edge e = iterator.next();
//				int vertexID = e.getDestination();
//				if(!visited[vertexID]) {
//					visited[vertexID] = true;
//					queue.offer(vertexID);
//				}
//			}
//
//			counter++;
//
//		}
//
//		if( verticesTracked == G.getVertexCount()) {
//			return vertices.toString();
//		}else {
//			return null;
//		}
	}

	public boolean isTree() {
//		System.out.println(ifConnectedThenBreadthFirstTraversal());
//		System.out.println("edge count: "+G.getEdgeCount());
//		System.out.println("vertex count - 1: "+(G.getVertexCount() - 1));
//		System.out.println(this.ifConnectedThenBreadthFirstTraversal() != null && G.getEdgeCount() == G.getVertexCount() - 1);
//		System.out.println();

		return this.ifConnectedThenBreadthFirstTraversal() != null && G.getEdgeCount() == (G.getVertexCount() - 1);
	}



}