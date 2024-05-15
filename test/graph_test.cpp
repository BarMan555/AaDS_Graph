#include <gtest/gtest.h>
#include "../include/graph.h"
#include <vector>

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

TEST(Graph, remove_vertex)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	ASSERT_TRUE(graph.remove_vertex(10));
}

TEST(Graph, vertices_test)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_vertex(100);
	ASSERT_EQ(graph.vertices(), std::vector<int>({10, 100}));
}