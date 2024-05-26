#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace Constants {
const int64_t kMaxLog = 20;
}

namespace GraphLib {
template <typename Vertex>
class Edge {
 public:
  Edge(const Vertex& begin, const Vertex& end) : begin_(begin), end_(end) {}

  auto GetBegin() const -> const Vertex& { return begin_; }

  auto GetEnd() const -> const Vertex& { return end_; }

 private:
  Vertex begin_;
  Vertex end_;
};

template <typename Vertex>
class AbstractGraph {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  virtual std::vector<EdgeT> GetOutgoingEdges(const VertexT& vertex) const = 0;
  virtual size_t NumVertices() const = 0;
  virtual size_t NumEdges() const = 0;
};

template <typename Vertex>
class AdjacencyListGraph : public AbstractGraph<Vertex> {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  AdjacencyListGraph(const std::vector<std::vector<EdgeT>>& adj_list)
      : adj_list_(adj_list) {}

  std::vector<EdgeT> GetOutgoingEdges(const VertexT& vertex) const override;

  size_t NumVertices() const override { return adj_list_.size(); }

  size_t NumEdges() const override;

 private:
  std::vector<std::vector<EdgeT>> adj_list_;
};

template <typename Vertex, bool is_root = false>
class Tree : public AdjacencyListGraph<Vertex> {
 private:
  Vertex root_;

 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  Tree(const std::vector<std::vector<EdgeT>>& adj_list, VertexT root = 0)
      : AdjacencyListGraph<VertexT>(adj_list), root_(root) {}
};

template <typename ElemType>
class DSU {
 private:
  std::unordered_map<ElemType, ElemType> parent_;

 public:
  DSU(ElemType size);

  ElemType FindSet(const ElemType& vertex);

  void UnionSets(const ElemType& vertex_1, const ElemType& vertex_2);
};

template <class Graph>
class AbstractDFSVisitor {
 public:
  virtual ~AbstractDFSVisitor() = default;
};

template <class Graph>
class DFSVisitor : public AbstractDFSVisitor<Graph> {
 public:
  using VertexT = typename Graph::VertexT;

  DFSVisitor(size_t size)
      : dist_(size), dp_(size, std::vector<int64_t>(Constants::kMaxLog, -1)) {}

  void WriteDP(VertexT vert_1, VertexT vert_2, VertexT dp) {
    dp_[vert_1][vert_2] = dp;
  }

  VertexT GetDP(VertexT vert_1, VertexT vert_2) { return dp_[vert_1][vert_2]; }

  void WriteDist(VertexT vert, VertexT dist) { dist_[vert] = dist; }

  VertexT GetDist(VertexT vert) { return dist_[vert]; }

 private:
  std::vector<VertexT> dist_;
  std::vector<std::vector<VertexT>> dp_;
};

template <typename Vertex>
void DFS(const GraphLib::Tree<Vertex>& graph,
         DFSVisitor<GraphLib::Tree<Vertex>>& visitor, Vertex point,
         Vertex depth, Vertex parent);

template <typename Vertex>
Vertex LCA(DFSVisitor<GraphLib::Tree<Vertex>>& visitor, Vertex vertex_1,
           Vertex vertex_2);

}  // namespace GraphLib

template <typename Vertex>
std::vector<typename GraphLib::AdjacencyListGraph<Vertex>::EdgeT>
GraphLib::AdjacencyListGraph<Vertex>::GetOutgoingEdges(
    const Vertex& vertex) const {
  if (static_cast<size_t>(vertex) < adj_list_.size()) {
    return adj_list_[vertex];
  }
  return {};
}

template <typename Vertex>
size_t GraphLib::AdjacencyListGraph<Vertex>::NumEdges() const {
  size_t count = 0;
  for (const auto& edges : adj_list_) {
    count += edges.size();
  }
  return count / 2;
}

template <typename ElemType>
GraphLib::DSU<ElemType>::DSU(ElemType size) {
  for (ElemType i = 0; i < size; ++i) {
    parent_[i] = i;
  }
}

template <typename ElemType>
ElemType GraphLib::DSU<ElemType>::FindSet(const ElemType& vertex) {
  if (vertex == parent_[vertex]) {
    return vertex;
  }
  return parent_[vertex] = FindSet(parent_[vertex]);
}

template <typename ElemType>
void GraphLib::DSU<ElemType>::UnionSets(const ElemType& vertex_1,
                                        const ElemType& vertex_2) {
  ElemType root_vertex_1 = FindSet(vertex_1);
  ElemType root_vertex_2 = FindSet(vertex_2);
  if (root_vertex_1 != root_vertex_2) {
    parent_[root_vertex_2] = root_vertex_1;
  }
}

template <typename Vertex>
void GraphLib::DFS(const GraphLib::Tree<Vertex, false>& graph,
                   GraphLib::DFSVisitor<GraphLib::Tree<Vertex, false>>& visitor,
                   Vertex point, Vertex depth, Vertex parent) {
  visitor.WriteDist(point, depth);
  visitor.WriteDP(point, 0, parent);
  for (int64_t i = 1; i < Constants::kMaxLog; ++i) {
    if (visitor.GetDP(point, i - 1) != -1) {
      visitor.WriteDP(point, i,
                      visitor.GetDP(visitor.GetDP(point, i - 1), i - 1));
    }
  }
  for (const auto& edge : graph.GetOutgoingEdges(point)) {
    if (edge.GetEnd() == parent) {
      continue;
    }
    DFS(graph, visitor, edge.GetEnd(), depth + 1, point);
  }
}

template <typename Vertex>
Vertex GraphLib::LCA(DFSVisitor<GraphLib::Tree<Vertex>>& visitor,
                     Vertex vertex_1, Vertex vertex_2) {
  if (visitor.GetDist(vertex_1) < visitor.GetDist(vertex_2)) {
    std::swap(vertex_1, vertex_2);
  }
  int64_t diff = visitor.GetDist(vertex_1) - visitor.GetDist(vertex_2);
  for (int64_t i = 0; diff; ++i) {
    if (diff & 1) {
      vertex_1 = visitor.GetDP(vertex_1, i);
    }
    diff >>= 1;
  }
  if (vertex_1 == vertex_2) {
    return vertex_1;
  }
  for (int64_t i = Constants::kMaxLog - 1; i >= 0; --i) {
    if (visitor.GetDP(vertex_1, i) != -1 &&
        visitor.GetDP(vertex_1, i) != visitor.GetDP(vertex_2, i)) {
      vertex_1 = visitor.GetDP(vertex_1, i);
      vertex_2 = visitor.GetDP(vertex_2, i);
    }
  }
  return visitor.GetDP(vertex_1, 0);
}

template <typename Vertex>
void ProcessRequests(const GraphLib::Tree<Vertex>& graph) {
  GraphLib::DFSVisitor<GraphLib::Tree<Vertex>> visitor(graph.NumVertices());
  GraphLib::DFS<int64_t>(graph, visitor, 0, 0, -1);
  int64_t num_proccess = 0;
  std::cin >> num_proccess;
  for (int64_t i = 0; i < num_proccess; ++i) {
    Vertex vertex_1 = 0;
    Vertex vertex_2 = 0;
    std::cin >> vertex_1 >> vertex_2;
    int64_t lca = LCA(visitor, vertex_1 - 1, vertex_2 - 1);
    std::cout << visitor.GetDist(vertex_1 - 1) + visitor.GetDist(vertex_2 - 1) -
                     2 * visitor.GetDist(lca)
              << '\n';
  }
}

template <typename Vertex>
auto Input(size_t num_vert) -> GraphLib::Tree<Vertex, false> {
  std::vector<std::vector<GraphLib::Edge<Vertex>>> edges(num_vert);
  Vertex begin = 0;
  Vertex end = 0;

  for (size_t i = 0; i < num_vert - 1; ++i) {
    std::cin >> begin >> end;
    edges[begin - 1].emplace_back(begin - 1, end - 1);
    edges[end - 1].emplace_back(end - 1, begin - 1);
  }

  return GraphLib::Tree<Vertex>(edges);
}

void SetupFastIO() {
  std::ios_base::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);
}

int main() {
  SetupFastIO();

  size_t num_vert = 0;
  std::cin >> num_vert;

  auto graph = Input<int64_t>(num_vert);

  ProcessRequests(graph);
}
