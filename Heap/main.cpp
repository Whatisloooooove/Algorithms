#include <iostream>
#include <vector>

class Heap {
 public:
  Heap() = default;

  Heap(std::vector<int64_t>&& vector) : body_(std::move(vector)) { Heapify(); }

  void Insert(int64_t index, int64_t num) {
    // Индекс передаётся для того ,чтобы DecreaseKey знал какой был по счёту
    // запрос
    body_.push_back(num);
    index_req_.push_back(index);
    SiftUp(body_.size() - 1);
  }

  void ExtractMin() {
    std::swap(body_.front(), body_.back());
    std::swap(index_req_.front(), index_req_.back());
    body_.pop_back();
    index_req_.pop_back();
    if (!body_.empty()) {
      SiftDown(0);
    }
  }

  const int64_t& GetMin() const { return body_[0]; }

  void DecreaseKey(int64_t ind, int64_t diff) {
    int32_t size = body_.size();
    for (int32_t i = 0; i < size; ++i) {
      if (index_req_[i] == ind) {
        body_[i] -= diff;
        SiftUp(i);
        break;
      }
    }
  }

 private:
  void Heapify() {
    for (uint32_t i = body_.size(); i > 0; --i) {
      SiftDown(i - 1);
    }
  }

  void SiftUp(uint32_t index) {
    while (index != 0) {
      uint32_t parent = (index - 1) / 2;
      if (body_[index] < body_[parent]) {
        std::swap(body_[index], body_[parent]);
        std::swap(index_req_[index], index_req_[parent]);
        index = parent;
      } else {
        break;
      }
    }
  }

  void SiftDown(uint32_t index) {
    while (2 * index <= body_.size()) {
      uint32_t left_child_ind = 2 * index + 1;
      uint32_t right_child_ind = 2 * index + 2;
      uint32_t min_ind = index;

      if (left_child_ind < body_.size() &&
          body_[left_child_ind] < body_[min_ind]) {
        min_ind = left_child_ind;
      }

      if (right_child_ind < body_.size() &&
          body_[right_child_ind] < body_[min_ind]) {
        min_ind = right_child_ind;
      }

      if (min_ind == index) {
        return;
      }
      std::swap(body_[index], body_[min_ind]);
      std::swap(index_req_[index], index_req_[min_ind]);

      index = min_ind;
    }
  }

  std::vector<int64_t> index_req_;
  std::vector<int64_t> body_;
};

void ProcessRequest(Heap& heap, std::string& request, int32_t req_index) {
  if (request == "insert") {
    int64_t new_elem;
    std::cin >> new_elem;
    heap.Insert(req_index, new_elem);
  } else if (request == "getMin") {
    std::cout << heap.GetMin() << '\n';
  } else if (request == "extractMin") {
    heap.ExtractMin();
  } else if (request == "decreaseKey") {
    int64_t ind;
    int64_t delta;
    std::cin >> ind >> delta;
    heap.DecreaseKey(ind, delta);
  }
}

int main() {
  std::ios_base::sync_with_stdio(false);
  Heap heap;
  int32_t num_requests;
  std::cin >> num_requests;
  for (int32_t req_index = 1; req_index <= num_requests; ++req_index) {
    std::string request;
    std::cin >> request;
    ProcessRequest(heap, request, req_index);
  }
}