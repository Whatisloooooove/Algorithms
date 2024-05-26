#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

namespace GraphLib {

constexpr int64_t kMaxPath = 2009000999;

template <typename Vertex, typename Weight>
struct Edge {
  Edge(const Vertex& begin, const Vertex& end, const Weight& weight)
      : begin_(begin), end_(end), weight_(weight) {}

  auto GetBegin() const -> const Vertex& { return begin_; }

  auto GetEnd() const -> const Vertex& { return end_; }

  auto GetWeight() const -> const Weight& { return weight_; }

 private:
  Vertex begin_;
  Vertex end_;
  Weight weight_;
};

template <typename Weight = int64_t, typename Vertex = int64_t>
class AbstractGraph {
 public:
  using VertexT = Vertex;
  using WeightT = Weight;
  using EdgeT = Edge<VertexT, WeightT>;

  virtual std::vector<EdgeT> GetOutgoingEdges(const Vertex& vertex) const = 0;
  virtual size_t NumVertices() const = 0;
  virtual size_t NumEdges() const = 0;
};

template <typename Weight = int64_t, typename Vertex = int64_t>
class AdjacencyListGraph : public AbstractGraph<Vertex> {
 public:
  using VertexT = Vertex;
  using WeightT = Weight;
  using EdgeT = Edge<VertexT, WeightT>;

  AdjacencyListGraph(const std::vector<std::vector<EdgeT>>& edges,
                     const size_t& num_vertex, const size_t& num_edges)
      : size_graph_(num_vertex), adjacent_(edges), num_edges_(num_edges) {}

  std::vector<EdgeT> GetOutgoingEdges(const Vertex& vertex) const override;

  size_t NumVertices() const override { return size_graph_; }

  size_t NumEdges() const override { return num_edges_; }

 private:
  size_t size_graph_;
  std::vector<std::vector<EdgeT>> adjacent_;
  size_t num_edges_;
};

template <class Graph>
class AbstractVisitor {
 public:
  virtual ~AbstractVisitor() = default;
};

template <class Graph>
class DFSVisitor : public AbstractVisitor<Graph> {
 public:
  using VertexT = typename Graph::VertexT;
  using Component = std::vector<VertexT>;

  std::vector<VertexT> FindShortPaths(const Graph& graph,
                                      VertexT& start_vertex);

  VertexT GetDist(const VertexT& vertex) const { return distance_[vertex]; }

  void WritePath(const VertexT& vertex, const VertexT& dist) {
    distance_[vertex] = dist;
  }

 private:
  using BaseClass = AbstractVisitor<Graph>;

  std::vector<VertexT> distance_;
};

template <class Graph, class Visitor>
void Dijkstra(const Graph& graph, const typename Graph::VertexT& begin_vertex,
              Visitor& visitor);

}  // namespace GraphLib

template <typename Weight, typename Vertex>
std::vector<typename GraphLib::AdjacencyListGraph<Weight, Vertex>::EdgeT>
GraphLib::AdjacencyListGraph<Weight, Vertex>::GetOutgoingEdges(
    const Vertex& vertex) const {
  if (vertex < int64_t(size_graph_)) {
    return adjacent_[vertex];
  }
  return {};
}

template <class Graph>
std::vector<typename Graph::VertexT>
GraphLib::DFSVisitor<Graph>::FindShortPaths(
    const Graph& graph, typename Graph::VertexT& start_vertex) {
  distance_.assign(graph.NumVertices(), GraphLib::kMaxPath);
  Dijkstra(graph, start_vertex, *this);
  return distance_;
}

template <class Graph, class Visitor>
void GraphLib::Dijkstra(const Graph& graph,
                        const typename Graph::VertexT& begin_vertex,
                        Visitor& visitor) {
  std::priority_queue<std::pair<int64_t, int64_t>,
                      std::vector<std::pair<int64_t, int64_t>>, std::greater<>>
      heap;
  heap.push({0, begin_vertex});
  while (!heap.empty()) {
    const auto [dist, vertex] = heap.top();
    heap.pop();
    if (visitor.GetDist(vertex) <= dist) {
      continue;
    }
    visitor.WritePath(vertex, dist);
    for (const auto& edge : graph.GetOutgoingEdges(vertex)) {
      if (visitor.GetDist(edge.GetEnd()) <= dist + edge.GetWeight()) {
        continue;
      }
      heap.push({dist + edge.GetWeight(), edge.GetEnd()});
    }
  }
}

auto InputGraph() -> GraphLib::AdjacencyListGraph<int64_t, int64_t> {
  int64_t num_vert = 0;
  int64_t num_edges = 0;
  std::cin >> num_vert >> num_edges;
  std::vector<std::vector<GraphLib::Edge<int64_t, int64_t>>> edges(num_vert);
  int64_t start_vertex = 0;
  int64_t end_vertex = 0;
  int64_t weight = 0;
  for (int64_t ind = 0; ind < num_edges; ++ind) {
    std::cin >> start_vertex >> end_vertex >> weight;
    edges[start_vertex].push_back(
        GraphLib::Edge<int64_t, int64_t>(start_vertex, end_vertex, weight));
    edges[end_vertex].push_back(
        GraphLib::Edge<int64_t, int64_t>(end_vertex, start_vertex, weight));
  }
  return GraphLib::AdjacencyListGraph<int64_t, int64_t>(edges, num_vert,
                                                        num_edges);
}

void PrintAns(const std::vector<int64_t>& paths) {
  for (const auto& path : paths) {
    std::cout << path << " ";
  }
  std::cout << '\n';
}

void SetupFastIO() {
  std::ios_base::sync_with_stdio(false);
  std::cin.tie(NULL);
  std::cout.tie(NULL);
}

int main() {
  SetupFastIO();
  int64_t num_map = 0;
  std::cin >> num_map;
  for (int64_t ind = 0; ind < num_map; ++ind) {
    GraphLib::AdjacencyListGraph<int64_t> graph = InputGraph();
    int64_t begin_vertex = 0;
    std::cin >> begin_vertex;
    GraphLib::DFSVisitor<GraphLib::AdjacencyListGraph<int64_t>> visitor;
    const auto& paths = visitor.FindShortPaths(graph, begin_vertex);
    PrintAns(paths);
  }
}
