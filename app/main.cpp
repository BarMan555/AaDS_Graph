#include "graph.h"
#include <iostream>

int main()
{
	using namespace GraphSpace;
	using namespace std;

	Graph<int, int> gr;
	gr.add_vertex(10);
	gr.add_vertex(50);
	gr.add_vertex(100);
	gr.add_edge(10, 50, 1);
	gr.add_edge(50, 100, 1);
	gr.print(10);

	gr.vector_walk(10);
	return 0;
}