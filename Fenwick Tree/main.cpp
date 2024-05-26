#include <cstdint>
#include <iostream>
#include <vector>

class FenwickTree3D {
 public:
  FenwickTree3D(int32_t num_square)
      : size_(num_square),
        tree_(num_square,
              std::vector<std::vector<int32_t>>(
                  num_square, std::vector<int32_t>(num_square, 0))) {}

  void Update(std::vector<int32_t>& point, int32_t delta) {
    for (int32_t i = point[0]; i < size_; i = SetFunc(i)) {
      for (int32_t j = point[1]; j < size_; j = SetFunc(j)) {
        for (int32_t k = point[2]; k < size_; k = SetFunc(k)) {
          tree_[i][j][k] += delta;
        }
      }
    }
  }

  int32_t GetSubsegmentSum(std::vector<int32_t>& point_1,
                           std::vector<int32_t>& point_2) {
    return GetPrefixSum(point_2[0], point_2[1], point_2[2]) -
           GetPrefixSum(point_1[0] - 1, point_2[1], point_2[2]) -
           GetPrefixSum(point_2[0], point_1[1] - 1, point_2[2]) -
           GetPrefixSum(point_2[0], point_2[1], point_1[2] - 1) +
           GetPrefixSum(point_1[0] - 1, point_1[1] - 1, point_2[2]) +
           GetPrefixSum(point_1[0] - 1, point_2[1], point_1[2] - 1) +
           GetPrefixSum(point_2[0], point_1[1] - 1, point_1[2] - 1) -
           GetPrefixSum(point_1[0] - 1, point_1[1] - 1, point_1[2] - 1);
  }

 private:
  int32_t size_;
  std::vector<std::vector<std::vector<int32_t>>> tree_;

  static int32_t SetFunc(int32_t value) { return value | (value + 1); }

  int32_t GetPrefixSum(int32_t x_pos, int32_t y_pos, int32_t z_pos) {
    int32_t result = 0;
    for (int32_t i = x_pos; i != -1; i = FindFunc(i) - 1) {
      for (int32_t j = y_pos; j != -1; j = FindFunc(j) - 1) {
        for (int32_t k = z_pos; k != -1; k = FindFunc(k) - 1) {
          result += tree_[i][j][k];
        }
      }
    }
    return result;
  }

  static int32_t FindFunc(int32_t value) { return value & (value + 1); }
};

void ProccessRequest(FenwickTree3D& fenwick_tree_3d) {
  while (true) {
    int32_t request = 0;
    std::cin >> request;
    if (request == 1) {
      std::vector<int32_t> point(3);
      int32_t update = 0;
      std::cin >> point[0] >> point[1] >> point[2] >> update;
      fenwick_tree_3d.Update(point, update);
    } else if (request == 2) {
      std::vector<int32_t> point_1(3);
      std::vector<int32_t> point_2(3);
      std::cin >> point_1[0] >> point_1[1] >> point_1[2] >> point_2[0] >>
          point_2[1] >> point_2[2];
      int32_t result = fenwick_tree_3d.GetSubsegmentSum(point_1, point_2);
      std::cout << result << "\n";
    } else if (request == 3) {
      break;
    }
  }
}

int main() {
  std::ios_base::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);

  int32_t num_square = 0;
  std::cin >> num_square;
  FenwickTree3D fenwick_tree_3d(num_square);
  ProccessRequest(fenwick_tree_3d);
}