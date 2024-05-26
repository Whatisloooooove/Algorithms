// https://brestprog.by/topics/connectivity/
// https://pastebin.com/dNR8HTjv
#include <iostream>
#include <span>
#include <unordered_map>
#include <vector>

namespace GraphTools {

enum class Colors { White = 0, Grey, Black };

template <typename Vertex>
struct Edge {
  Edge(const Vertex& from, const Vertex& to) : from_(from), to_(to) {}

  auto GetStart() const -> const Vertex& { return from_; }

  auto GetEnd() const -> const Vertex& { return to_; }

 private:
  Vertex from_;
  Vertex to_;
};

template <typename Vertex = int32_t>
class AbstractGraph {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  virtual std::span<const EdgeT> GetOutgoingEdges(
      const Vertex& vertex) const = 0;
  virtual size_t NumVertices() const = 0;
  virtual size_t NumEdges() const = 0;
};

template <typename Vertex = int32_t>
class Graph : public AbstractGraph<Vertex> {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  Graph(const std::vector<EdgeT>& edges, const size_t& num_vertex);

  std::span<const EdgeT> GetOutgoingEdges(const Vertex& vertex) const override;

  size_t NumVertices() const override { return size_graph_; }

  size_t NumEdges() const override;

 private:
  size_t size_graph_;
  std::unordered_map<VertexT, std::vector<EdgeT>> adjacent_;
};

template <class Graph>
class AbstractDFSVisitor {
 public:
  virtual void DiscoverVertex(const typename Graph::VertexT& vertex) = 0;
  virtual void FinishVertex(const typename Graph::VertexT& vertex) = 0;
  virtual Colors GetColor(const typename Graph::VertexT& vertex) = 0;
  virtual ~AbstractDFSVisitor() = default;

 protected:
  std::unordered_map<typename Graph::VertexT, Colors> colors_;
};

template <class Graph>
class ComponentsDFSVisitor : public AbstractDFSVisitor<Graph> {
 public:
  using VertexT = typename Graph::VertexT;
  using Component = std::vector<VertexT>;

  void DiscoverVertex(const VertexT& vertex) override;

  void FinishVertex(const VertexT& vertex) override {
    AbstractDFSVisitor<Graph>::colors_[vertex] = Colors::Black;
  }

  Colors GetColor(const VertexT& vertex) override;

  const std::vector<Component>& GetComponents() const { return components_; }

  void AddComponent();

 private:
  std::vector<Component> components_;
  Component current_component_;
};

template <class Graph, class Visitor>
void DFS(Graph& graph, const typename Graph::VertexT& vertex, Visitor& visitor);

}  // namespace GraphTools

template <typename Vertex>
GraphTools::Graph<Vertex>::Graph(const std::vector<EdgeT>& edges,
                                 const size_t& num_vertex)
    : size_graph_(num_vertex) {
  for (const auto& edge : edges) {
    adjacent_[edge.GetStart()].push_back(edge);
    adjacent_[edge.GetEnd()].push_back(EdgeT(edge.GetEnd(), edge.GetStart()));
  }
}

template <typename Vertex>
std::span<const typename GraphTools::Graph<Vertex>::EdgeT>
GraphTools::Graph<Vertex>::GetOutgoingEdges(const Vertex& vertex) const {
  if (auto it = adjacent_.find(vertex); it != adjacent_.end()) {
    return it->second;
  }
  return {};
}

template <typename Vertex>
size_t GraphTools::Graph<Vertex>::NumEdges() const {
  size_t count_edges = 0;
  for (const auto& edge_list : adjacent_) {
    count_edges += edge_list.second.size();
  }
  return count_edges / 2;
}

template <class Graph>
void GraphTools::ComponentsDFSVisitor<Graph>::DiscoverVertex(
    const VertexT& vertex) {
  AbstractDFSVisitor<Graph>::colors_[vertex] = Colors::Grey;
  current_component_.push_back(vertex);
}

template <class Graph>
typename GraphTools::Colors GraphTools::ComponentsDFSVisitor<Graph>::GetColor(
    const VertexT& vertex) {
  auto it = AbstractDFSVisitor<Graph>::colors_.find(vertex);
  if (it != AbstractDFSVisitor<Graph>::colors_.end()) {
    return it->second;
  }
  return Colors::White;
}

template <class Graph>
void GraphTools::ComponentsDFSVisitor<Graph>::AddComponent() {
  components_.push_back(std::move(current_component_));
  current_component_.clear();
}

template <class Graph, class Visitor>
void GraphTools::DFS(Graph& graph, const typename Graph::VertexT& vertex,
                     Visitor& visitor) {
  visitor.DiscoverVertex(vertex);
  for (const auto& outgoing_edge : graph.GetOutgoingEdges(vertex)) {
    const auto& neighbour = outgoing_edge.GetEnd();
    if (visitor.GetColor(neighbour) == Colors::White) {
      DFS(graph, neighbour, visitor);
    }
  }
  visitor.FinishVertex(vertex);
}

GraphTools::Graph<> Input() {
  size_t num_points = 0;
  size_t num_edges = 0;
  std::cin >> num_points >> num_edges;

  std::vector<GraphTools::Edge<int32_t>> edges;
  edges.reserve(num_edges);

  int32_t from = 0;
  int32_t to = 0;
  for (size_t ind = 0; ind < num_edges; ++ind) {
    std::cin >> from >> to;
    edges.emplace_back(from - 1, to - 1);
  }

  return GraphTools::Graph<>(edges, num_points);
}

void PrintAns(const std::vector<std::vector<int32_t>>& components) {
  int32_t num_comp = components.size();
  std::cout << num_comp << '\n';
  for (int32_t ind = 0; ind < num_comp; ++ind) {
    std::cout << components[ind].size() << '\n';
    for (auto point : components[ind]) {
      std::cout << point + 1 << ' ';
    }
    std::cout << '\n';
  }
}

int main() {
  GraphTools::Graph<> graph = Input();
  GraphTools::ComponentsDFSVisitor<GraphTools::Graph<>> visitor;

  for (size_t vertex = 0; vertex < graph.NumVertices(); ++vertex) {
    if (visitor.GetColor(vertex) == GraphTools::Colors::White) {
      GraphTools::DFS(graph, vertex, visitor);
      visitor.AddComponent();
    }
  }
  const auto& components = visitor.GetComponents();
  PrintAns(components);
}
