#include <gtest/gtest.h>
#include "../include/graph.h"
#include <vector>
#include <string>

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

TEST(Graph, add_edge)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_edge(10, 10, 1);
	ASSERT_TRUE(graph.has_edge(10, 10));
}

TEST(Graph, remove_edge_1)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_vertex(11);
	graph.add_vertex(12);
	graph.add_edge(10, 11, 1);
	graph.add_edge(10, 12, 2);
	graph.remove_edge(10, 11);
	ASSERT_FALSE(graph.has_edge(10, 11));
}

TEST(Graph, remove_edge_2)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_vertex(11);
	graph.add_vertex(12);
	graph.add_edge(10, 11, 1);
	graph.add_edge(10, 12, 2);
	Graph<int, int>::Edge edge(10, 11, 1);
	graph.remove_edge(edge);
	ASSERT_FALSE(graph.has_edge(edge));
}

TEST(Graph, has_edge_1)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_vertex(15);
	graph.add_edge(10, 15, 1);
	ASSERT_TRUE(graph.has_edge(10, 15));
}

TEST(Graph, has_edge_2)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_vertex(15);
	graph.add_edge(10, 15, 1);
	Graph<int, int>::Edge edge(10, 15, 1);
	ASSERT_TRUE(graph.has_edge(edge));
}

TEST(Graph, order)
{
	Graph<int, int> graph;
	graph.add_vertex(10);
	graph.add_vertex(15);
	ASSERT_EQ(graph.order(), 2);
}

TEST(Graph, degree)
{
	Graph<int, int> graph;
	graph.add_vertex(15);
	graph.add_edge(15, 15, 1);
	graph.add_edge(15, 15, 2);
	graph.add_edge(15, 15, 3);
	ASSERT_EQ(graph.degree(15), 3);
}

TEST(task, opt_warehouse)
{
	Graph<std::string, int> graph;
	graph.add_vertex("A");
	graph.add_vertex("B");
	graph.add_vertex("C");
	graph.add_vertex("D");

	graph.add_edge("A", "B", 1.0);
	graph.add_edge("A", "C", 4.0);
	graph.add_edge("B", "C", 2.0);
	graph.add_edge("C", "D", 1.0);

	std::string warehouse = find_optimal_warehouse(graph);
	ASSERT_EQ(warehouse, "C");
}