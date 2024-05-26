#include <iostream>
#include <vector>

std::vector<int32_t> Merge(std::vector<int32_t>& left,
                           std::vector<int32_t>& right) {
  int32_t ind_1 = 0;
  int32_t ind_2 = 0;
  int32_t left_size = left.size();
  int32_t right_size = right.size();
  std::vector<int32_t> result;
  while (ind_1 < left_size && ind_2 < right_size) {
    if (left[ind_1] <= right[ind_2]) {
      result.push_back(left[ind_1]);
      ++ind_1;
    } else {
      result.push_back(right[ind_2]);
      ++ind_2;
    }
  }
  while (ind_1 < left_size) {
    result.push_back(left[ind_1]);
    ++ind_1;
  }
  while (ind_2 < right_size) {
    result.push_back(right[ind_2]);
    ++ind_2;
  }
  return result;
}

void InputOneVector(std::vector<std::vector<int32_t>>& all_vectors) {
  int32_t size_sub = 0;
  std::cin >> size_sub;
  std::vector<int32_t> sub(size_sub);
  for (int32_t ind_elem = 0; ind_elem < size_sub; ++ind_elem) {
    std::cin >> sub[ind_elem];
  }
  all_vectors.push_back(sub);
}

std::vector<std::vector<int32_t>> InputBigVector() {
  int32_t num_arr = 0;
  std::cin >> num_arr;
  std::vector<std::vector<int32_t>> all_vectors;
  for (int32_t ind_vec = 0; ind_vec < num_arr; ++ind_vec) {
    InputOneVector(all_vectors);
  }
  return all_vectors;
}

std::vector<std::vector<int32_t>> MergeAllVectors(
    std::vector<std::vector<int32_t>>& all_vectors) {
  int32_t size = all_vectors.size();
  if (size == 1) {
    return all_vectors;
  }
  std::vector<std::vector<int32_t>> merge_vec;
  for (int32_t ind = 0; ind < size; ind += 2) {
    if ((ind + 1) < size) {
      std::vector<int32_t> res = Merge(all_vectors[ind], all_vectors[ind + 1]);
      merge_vec.push_back(res);
    } else {
      merge_vec.push_back(all_vectors[ind]);
    }
  }
  return MergeAllVectors(merge_vec);
}

void Output(std::vector<int32_t>& res) {
  int32_t size = res.size();
  for (int32_t ind = 0; ind < size; ++ind) {
    std::cout << res[ind] << ' ';
  }
}

int main() {
  std::vector<std::vector<int32_t>> all_vectors = InputBigVector();
  std::vector<std::vector<int32_t>> res = MergeAllVectors(all_vectors);
  Output(res[0]);
}