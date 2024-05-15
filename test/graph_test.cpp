#include <gtest/gtest.h>
#include "../include/graph.h"

using namespace GraphSpace;

TEST(Graph, has_vertex)
{
	Graph<int, int> graph;
	ASSERT_FALSE(graph.has_vertex(10));
}

TEST(Graph, add_vertex)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	ASSERT_TRUE(graph.has_vertex(10));
}
