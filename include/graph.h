#pragma once
#include <vector>
#include <set>
#include <unordered_map>
#include <list>

namespace GraphSpace
{
	template<typename Vertex, typename Distance = double>
	class Graph
	{

		struct Edge
		{
			Vertex from, to;
			Distance distance;
			Edge(Vertex from, Vertex to, Distance distance) :from(from), to(to), distance(distance) {}
		};
		std::set<Vertex> _vertices;
		std::unordered_map<Vertex, std::list<Edge>> _edges;

	public:

		bool has_vertex(const Vertex& v) const;
		void add_vertex(const Vertex& v);
		bool remove_vertex(const Vertex& v);
		std::vector<Vertex> vertices() const;

		void add_edge(const Vertex& from, const Vertex& to, const Distance& d);
		bool remove_edge(const Vertex& from, const Vertex& to);
		bool remove_edge(const Edge& e); //c ������ ����������
		bool has_edge(const Vertex& from, const Vertex& to) const;
		bool has_edge(const Edge& e) const; //c ������ ���������� � Edge

		//��������� ���� �����, ��������� �� �������
		std::vector<Edge> edges(const Vertex& vertex);

		size_t order() const; //������� 
		size_t degree(const Vertex& v) const; //������� �������

		//����� ����������� ����
		std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const;
		//�����
		std::vector<Vertex>  walk(const Vertex& start_vertex)const;
	};

	template<typename Vertex, typename Distance>
	bool GraphSpace::Graph<Vertex, Distance>::has_vertex(const Vertex& v) const
	{
		if (_vertices.find(v) != _vertices.end()) return true;
		return false;
	}

	template<typename Vertex, typename Distance>
	void GraphSpace::Graph<Vertex, Distance>::add_vertex(const Vertex& v)
	{
		if (has_vertex(v)) return;

		_vertices.insert(v);
		_edges.insert({ v, {} });
	}
}