#pragma once
#include <algorithm>
#include <set>
#include <unordered_map>
#include <vector>
#include <queue>

namespace GraphSpace
{
	template<typename Vertex, typename Distance = double>
	class Graph
	{
	public:
		struct Edge
		{
			Vertex from;
			Vertex to;
			Distance distance;
			Edge(Vertex from, Vertex to, Distance distance) :from(from), to(to), distance(distance) {}
		};
	private:
		std::set<Vertex> _vertices;
		std::unordered_map<Vertex, std::vector<Edge>> _edges;

	public:

		bool has_vertex(const Vertex& v) const;
		void add_vertex(const Vertex& v);
		bool remove_vertex(const Vertex& v);
		std::vector<Vertex> vertices() const;

		void add_edge(const Vertex& from, const Vertex& to, const Distance& d);
		bool remove_edge(const Vertex& from, const Vertex& to);
		bool remove_edge(const Edge& e); //c учетом рассто€ни€
		bool has_edge(const Vertex& from, const Vertex& to) const;
		bool has_edge(const Edge& e) const; //c учетом рассто€ни€ в Edge

		//получение всех ребер, выход€щих из вершины
		std::vector<Edge> edges(const Vertex& vertex)
		{
			return _edges[vertex];
		}

		size_t order() const; //пор€док 
		size_t degree(const Vertex& v) const; //степень вершины

		//поиск кратчайшего пути
		std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const
		{
			if (!has_vertex(from) || !has_vertex(to)) {
				throw std::invalid_argument("Vertex not found in the graph");
			}

			std::unordered_map<Vertex, Distance> min_weight;
			for (const Vertex& v : _vertices) {
				min_weight[v] = std::numeric_limits<Distance>::max();
			}
			min_weight[from] = 0;


			std::priority_queue<std::pair<Distance, Vertex>> q;
			q.push({ 0, from });

			std::unordered_map<Vertex, Vertex> previous;
			while (!q.empty()) {
				std::pair<Distance, Vertex> temp = q.top();
				q.pop();

				Distance curr_weight = temp.first;
				Vertex curr_vertex = temp.second;

				if (curr_weight > min_weight[curr_vertex]) {
					continue;
				}
				if (_edge.find(curr_vertex) == _edge.end())
					continue;
				for (const Edge& edge : _edge.at(curr_vertex)) {
					Distance new_weight = curr_weight + edge.distance;
					if (new_weight < min_weight[edge.to]) {
						previous[edge.to] = curr_vertex;
						min_weight[edge.to] = new_weight;
						q.push({ new_weight, edge.to });
					}
				}
			}

			if (min_weight[to] == std::numeric_limits<Distance>::max()) {
				return {};
			}
		
			std::vector<Edge> path;
			Vertex current = to;
			while (current != from) {
				Vertex parent = previous[current];
				Distance weight = min_weight[current] - min_weight[parent];
				path.push_back(Edge{ parent, current, weight });
				current = parent;
			}
			std::reverse(path.begin(), path.end());
			return path;
		}

		//обход
		std::vector<Vertex>  walk(const Vertex& start_vertex)const;
	};

	template<typename Vertex, typename Distance>
	bool Graph<Vertex, Distance>::has_vertex(const Vertex& v) const
	{
		return _vertices.find(v) != _vertices.end();
	}

	template<typename Vertex, typename Distance>
	void Graph<Vertex, Distance>::add_vertex(const Vertex& v)
	{
		if (has_vertex(v)) return;

		_vertices.insert(v);
		_edges.insert({ v, {} });
	}

	template<typename Vertex, typename Distance>
	bool Graph<Vertex, Distance>::remove_vertex(const Vertex& v)
	{
		if (!has_vertex(v))
			return false;
		_vertices.erase(v);
		_edges.erase(v);

		for (auto& vert : _vertices) {
			auto& e = _edges[vert];
			for (auto& it : e) {
				if (it.to == v) {
					it.to = vert;
				};
			};
		}
		return true;
	}

	template<typename Vertex, typename Distance>
	std::vector<Vertex> Graph<Vertex, Distance>::vertices() const
	{
		std::vector<Vertex> result;
		for (auto& v : _vertices)
		{
			result.push_back(v);
		}
		return result;
	}

	template<typename Vertex, typename Distance>
	void GraphSpace::Graph<Vertex, Distance>::add_edge(const Vertex& from, const Vertex& to, const Distance& d)
	{
		Edge new_edge(from, to, d);
		if (has_edge(new_edge))
			return;
		if (!has_vertex(from))
			_vertices.insert(from);
		if (!has_vertex(to))
			_vertices.insert(to);
		_edges[from].push_back(new_edge);
	}

	template<typename Vertex, typename Distance>
	bool GraphSpace::Graph<Vertex, Distance>::remove_edge(const Vertex& from, const Vertex& to)
	{
		auto it = _edges.find(from);
		if (it == _edges.end()) return false;
		auto& edges = it->second;
		edges.erase(std::remove_if(edges.begin(), edges.end(), [&](const Edge& e) { return e.to == to; }), edges.end());
		return true;
	}

	template<typename Vertex, typename Distance>
	bool GraphSpace::Graph<Vertex, Distance>::remove_edge(const Edge& e)
	{
		if (!has_edge(e)) return false;
		auto it = _edges.find(e.from);
		auto& edges = it->second;
		edges.erase(std::remove_if(edges.begin(), edges.end(), [&](const Edge& ed) { return e.to == ed.to && e.distance == ed.distance; }), edges.end());
		return true;
	}

	template<typename Vertex, typename Distance>
	bool GraphSpace::Graph<Vertex, Distance>::has_edge(const Vertex& from, const Vertex& to) const
	{
		auto it = _edges.find(from);
		if (it == _edges.end())
			return false;
		auto& edges_vec = it->second;
		for (auto& e : edges_vec) 
		{
			if (e.to == to)
				return true;
		}
		return false;
	}	

	template<typename Vertex, typename Distance>
	bool GraphSpace::Graph<Vertex, Distance>::has_edge(const Edge& e) const
	{
		if (!has_edge(e.from, e.to))
			return false;
		auto it = _edges.find(e.from);
		auto& edges_vec = it->second;
		for (auto& edge : edges_vec) 
		{
			if (edge.to == e.to) 
			{
				if (e.distance == edge.distance) 
					return true;
			}
		}
		return false;
	}

	template<typename Vertex, typename Distance>
	size_t GraphSpace::Graph<Vertex, Distance>::order() const
	{
		return _vertices.size();
	}

	template<typename Vertex, typename Distance>
	size_t GraphSpace::Graph<Vertex, Distance>::degree(const Vertex& v) const
	{
		if (!has_vertex(v)) return 0;
		auto it = _edges.find(v);
		auto& edges = it->second;
		return edges.size();
	}
}