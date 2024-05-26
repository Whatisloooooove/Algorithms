#include <algorithm>
#include <iostream>
#include <vector>

bool CanEscape(int32_t mid, int32_t num_set, std::vector<int32_t>& places) {
  int32_t count = 1;
  int32_t prev = places[0];
  for (size_t ind = 1; ind < places.size(); ++ind) {
    if (places[ind] - prev >= mid) {
      count++;
      prev = places[ind];
    }
  }
  return count >= num_set;
}

int32_t CalcMinDist(int32_t num_set, std::vector<int32_t>& places) {
  int32_t left = 0;
  int32_t right = places.back() - places.front();
  int32_t mid = 0;
  int32_t res = 0;
  while (left <= right) {
    mid = left + (right - left) / 2;
    if (CanEscape(mid, num_set, places)) {
      res = mid;
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  return res;
}

void Input(std::vector<int32_t>& places, int32_t& num_set) {
  int32_t num_places = 0;
  int32_t point = 0;
  std::cin >> num_places >> num_set;
  for (int ind = 0; ind != num_places; ++ind) {
    std::cin >> point;
    places.push_back(point);
  }
}

void Output(int32_t result) { std::cout << result << '\n'; }

int main() {
  int32_t num_set = 0;
  std::vector<int32_t> places;
  Input(places, num_set);
  int32_t result = CalcMinDist(num_set, places);
  Output(result);
}