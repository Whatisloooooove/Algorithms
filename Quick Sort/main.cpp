#include <functional>
#include <iostream>
#include <random>
#include <vector>

template <typename Iterator>
Iterator SelectPivot(Iterator begin, Iterator end) {
  std::mt19937 gen(std::random_device{}());
  std::uniform_int_distribution<> dis(0, std::distance(begin, end) - 1);

  std::advance(begin, dis(gen));
  std::iter_swap(begin, end);
  return end;
}

template <typename Iterator, typename Comparator>
Iterator Partition(Iterator left, Iterator right, Comparator comp) {
  Iterator pivot = SelectPivot(left, right);
  Iterator cop_left = left;
  Iterator cop_right = right;

  while (cop_left <= cop_right) {
    while (comp(*cop_left, *pivot)) {
      ++cop_left;
    }
    while (comp(*pivot, *cop_right)) {
      --cop_right;
    }
    if (cop_left >= cop_right) {
      break;
    }
    std::swap(*cop_left, *cop_right);
    --cop_right;
    ++cop_left;
  }

  return cop_right;
}

template <typename Iterator, typename Comparator>
void QuickSort(Iterator left, Iterator right, Comparator comp) {
  if (left >= right) {
    return;
  }
  Iterator ind = Partition(left, right, comp);
  QuickSort(left, ind, comp);
  QuickSort(ind + 1, right, comp);
}

void Input(std::vector<int64_t>& arr) {
  int32_t size = 0;
  std::cin >> size;
  int64_t num = 0;
  for (int32_t ind = 0; ind < size; ind++) {
    std::cin >> num;
    arr.push_back(num);
  }
}

void Output(std::vector<int64_t>& arr) {
  int32_t size = arr.size();
  for (int32_t ind = 0; ind < size; ind++) {
    std::cout << arr[ind] << ' ';
  }
}

int main() {
  std::vector<int64_t> arr;
  Input(arr);
  auto custom_comparator = [](const int64_t& num_1, const int64_t& num_2) {
    return num_1 < num_2;
  };

  QuickSort(arr.begin(), arr.end() - 1, custom_comparator);
  Output(arr);
}
