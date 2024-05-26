// http://e-maxx.ru/algo/mst_kruskal_with_dsu
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace GraphLib {

template <typename Vertex>
struct Edge {
 public:
  Edge(const Vertex& begin, const Vertex& end) : begin_(begin), end_(end) {}

  auto GetBegin() const -> const Vertex& { return begin_; }

  auto GetEnd() const -> const Vertex& { return end_; }

 private:
  Vertex begin_;
  Vertex end_;
};

template <typename Vertex, typename Weight>
struct WeightedEdge : public Edge<Vertex> {
 public:
  WeightedEdge(const Vertex& begin, const Vertex& end, const Weight& weight)
      : Edge<Vertex>(begin, end), weight_(weight) {}

  auto GetWeight() const -> const Weight& { return weight_; }

  void SetWeight(const Weight& weight) { weight_ = weight; }

 private:
  Weight weight_;
};

template <typename Vertex = int64_t>
class AbstractGraph {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  virtual size_t NumVertices() const = 0;
  virtual size_t NumEdges() const = 0;
};

template <typename Vertex = int64_t>
class AdjacencyListGraph : public AbstractGraph<Vertex> {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  AdjacencyListGraph(const std::vector<std::vector<EdgeT>>& edges,
                     const size_t& num_vertex)
      : size_graph_(num_vertex), adjacent_(edges) {}

  std::vector<EdgeT> GetOutgoingEdges(const Vertex& vertex) const;

  size_t NumVertices() const override { return size_graph_; }

  size_t NumEdges() const override;

 private:
  size_t size_graph_;
  std::vector<std::vector<EdgeT>> adjacent_;
};
template <typename Vertex, typename Weight>
class WeightedTree;

template <typename ElemType>
class DSU;

template <typename Vertex = int64_t, typename Weight = int64_t>
class WeightedAdjacencyGraph : public AbstractGraph<Vertex> {
 public:
  using VertexT = Vertex;
  using WeightT = Weight;
  using WeightedEdgeT = WeightedEdge<VertexT, WeightT>;

  WeightedAdjacencyGraph(const std::vector<std::vector<WeightedEdgeT>>& edges,
                         const size_t& num_vertex)
      : size_graph_(num_vertex), adjacent_(edges) {}

  std::vector<WeightedEdgeT> GetOutgoingEdges(const Vertex& vertex) const;

  size_t NumVertices() const override { return size_graph_; }

  size_t NumEdges() const override;

  WeightedTree<Vertex, Weight> CalculateMST() const;

 private:
  size_t size_graph_;
  std::vector<std::vector<WeightedEdgeT>> adjacent_;
};

template <typename Vertex = int64_t, typename Weight = int64_t>
class WeightedTree : public AbstractGraph<Vertex> {
 public:
  using VertexT = Vertex;
  using WeightT = Weight;
  using WeightedEdgeT = WeightedEdge<VertexT, WeightT>;

  WeightedTree(const size_t& num_vertex) : size_graph_(num_vertex) {}

  size_t NumVertices() const override { return size_graph_; }

  size_t NumEdges() const override { return adjacent_.size(); };

  std::vector<WeightedEdgeT> GetEdges() const { return adjacent_; }

  void Insert(const WeightedEdgeT& edge) { adjacent_.push_back(edge); }

 private:
  size_t size_graph_;
  std::vector<WeightedEdgeT> adjacent_;
};

template <typename ElemType>
class DSU {
  std::unordered_map<ElemType, ElemType> parent_;

 public:
  DSU(ElemType size) {
    for (ElemType i = 0; i < size; ++i) {
      parent_[i] = i;
    }
  }

  ElemType FindSet(const ElemType& vertex);

  void UnionSets(const ElemType& vertex_1, const ElemType& vertex_2);
};

template <typename Vertex, typename Weight>
Weight KruskalAlgorithm(
    const GraphLib::WeightedAdjacencyGraph<Vertex, Weight>& graph);
}  // namespace GraphLib

template <typename Vertex>
std::vector<typename GraphLib::AdjacencyListGraph<Vertex>::EdgeT>
GraphLib::AdjacencyListGraph<Vertex>::GetOutgoingEdges(
    const Vertex& vertex) const {
  if (static_cast<size_t>(vertex) < size_graph_) {
    return adjacent_[vertex];
  }
  return {};
}

template <typename Vertex>
size_t GraphLib::AdjacencyListGraph<Vertex>::NumEdges() const {
  size_t count = 0;
  for (const auto& edges : adjacent_) {
    count += edges.size();
  }
  return count / 2;
}

template <typename Vertex, typename Weight>
std::vector<
    typename GraphLib::WeightedAdjacencyGraph<Vertex, Weight>::WeightedEdgeT>
GraphLib::WeightedAdjacencyGraph<Vertex, Weight>::GetOutgoingEdges(
    const Vertex& vertex) const {
  if (static_cast<size_t>(vertex) < size_graph_) {
    return adjacent_[vertex];
  }
  return {};
}

template <typename Vertex, typename Weight>
size_t GraphLib::WeightedAdjacencyGraph<Vertex, Weight>::NumEdges() const {
  size_t count = 0;
  for (const auto& edges : adjacent_) {
    count += edges.size();
  }
  return count / 2;
}

template <typename Vertex, typename Weight>
GraphLib::WeightedTree<Vertex, Weight>
GraphLib::WeightedAdjacencyGraph<Vertex, Weight>::CalculateMST() const {
  using WeightedEdgeT =
      GraphLib::WeightedAdjacencyGraph<Vertex, Weight>::WeightedEdgeT;
  std::vector<WeightedEdgeT> edges;
  for (size_t vertex = 0; vertex < size_graph_; ++vertex) {
    for (const auto& edge : adjacent_[vertex]) {
      edges.push_back(edge);
    }
  }
  std::sort(edges.begin(), edges.end(),
            [](const WeightedEdgeT& num_1, const WeightedEdgeT& num_2) {
              return num_1.GetWeight() < num_2.GetWeight();
            });

  GraphLib::DSU<Vertex> dsu(size_graph_);
  WeightedTree<Vertex, Weight> mst(size_graph_);
  for (const auto& edge : edges) {
    if (dsu.FindSet(edge.GetBegin()) != dsu.FindSet(edge.GetEnd())) {
      mst.Insert(edge);
      dsu.UnionSets(edge.GetBegin(), edge.GetEnd());
    }
  }

  return mst;
}

template <typename Vertex>
Vertex GraphLib::DSU<Vertex>::FindSet(const Vertex& vertex) {
  if (vertex == parent_[vertex]) {
    return vertex;
  }
  return parent_[vertex] = FindSet(parent_[vertex]);
}

template <typename Vertex>
void GraphLib::DSU<Vertex>::UnionSets(const Vertex& vertex_1,
                                      const Vertex& vertex_2) {
  Vertex root_vertex_1 = FindSet(vertex_1);
  Vertex root_vertex_2 = FindSet(vertex_2);
  if (root_vertex_1 != root_vertex_2) {
    parent_[root_vertex_2] = root_vertex_1;
  }
}

template <typename Vertex, typename Weight>
Weight GraphLib::KruskalAlgorithm(
    const GraphLib::WeightedAdjacencyGraph<Vertex, Weight>& graph) {
  const auto& mst = graph.CalculateMST();

  Weight count_weight = 0;
  for (const auto& edge : mst.GetEdges()) {
    count_weight += edge.GetWeight();
  }

  return count_weight;
}

template <typename Vertex, typename Weight>
auto Input(size_t num_vert, size_t num_edges)
    -> GraphLib::WeightedAdjacencyGraph<Vertex, Weight> {
  std::vector<std::vector<GraphLib::WeightedEdge<Vertex, Weight>>> edges(
      num_vert);
  Vertex begin = 0;
  Vertex end = 0;
  Weight weight = 0;

  for (size_t i = 0; i < num_edges; ++i) {
    std::cin >> begin >> end >> weight;
    edges[begin - 1].emplace_back(begin, end, weight);
    edges[end - 1].emplace_back(end, begin, weight);
  }

  return GraphLib::WeightedAdjacencyGraph<Vertex, Weight>(edges, num_vert);
}

void SetupFastIO() {
  std::ios_base::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);
}

int main() {
  SetupFastIO();

  size_t num_vert = 0;
  size_t num_edges = 0;
  std::cin >> num_vert >> num_edges;

  auto graph = Input<int64_t, int64_t>(num_vert, num_edges);

  std::cout << GraphLib::KruskalAlgorithm<int64_t, int64_t>(graph) << '\n';
}
