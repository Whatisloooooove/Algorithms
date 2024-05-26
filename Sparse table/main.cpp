#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

struct SparseTable {
  std::vector<std::vector<int32_t>> table;
  std::vector<int32_t> numbs;

  SparseTable(const std::vector<int32_t>& arr) : numbs(std::move(arr)) {
    int32_t size = numbs.size();
    int32_t pow = std::log2(size) + 1;

    table.resize(size, std::vector<int32_t>(pow));

    for (int32_t ind = 0; ind < size; ++ind) {
      table[ind][0] = ind;
    }

    for (int32_t ind_1 = 1; std::pow(2, ind_1) <= size; ++ind_1) {
      for (int32_t ind_2 = 0; ind_2 + std::pow(2, ind_1) - 1 < size; ++ind_2) {
        if (numbs[table[ind_2][ind_1 - 1]] <
            numbs[table[ind_2 + std::pow(2, ind_1 - 1)][ind_1 - 1]]) {
          table[ind_2][ind_1] = table[ind_2][ind_1 - 1];
        } else {
          table[ind_2][ind_1] =
              table[ind_2 + std::pow(2, ind_1 - 1)][ind_1 - 1];
        }
      }
    }
  }

  int32_t FindMin(int32_t left, int32_t right) {
    int32_t pow = std::log2(right - left + 1);
    if (numbs[table[left][pow]] <
        numbs[table[right - std::pow(2, pow) + 1][pow]]) {
      return table[left][pow];
    }
    return table[right - std::pow(2, pow) + 1][pow];
  }

  int32_t FindSecondStat(int32_t left, int32_t right) {
    int32_t first_stat = FindMin(left, right);
    if (first_stat == left) {
      return numbs[FindMin(left + 1, right)];
    }
    if (first_stat == right) {
      return numbs[FindMin(left, right - 1)];
    }
    return std::min(numbs[FindMin(left, first_stat - 1)],
                    numbs[FindMin(first_stat + 1, right)]);
  }
};

std::vector<int32_t> Input(int32_t& num_req) {
  int32_t size_seq = 0;
  std::cin >> size_seq >> num_req;
  std::vector<int32_t> sequence(size_seq);
  for (int32_t ind = 0; ind < size_seq; ++ind) {
    std::cin >> sequence[ind];
  }
  return sequence;
}

void ProccessQuerry(SparseTable& sparse_table, int32_t num_req) {
  for (int32_t ind = 0; ind < num_req; ++ind) {
    int32_t left = 0;
    int32_t right = 0;
    std::cin >> left >> right;
    std::cout << sparse_table.FindSecondStat(left - 1, right - 1) << '\n';
  }
}

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);
  int32_t num_req = 0;
  std::vector<int32_t> arr = Input(num_req);
  SparseTable sparse_table(arr);
  ProccessQuerry(sparse_table, num_req);
}