#include <iostream>
#include <vector>

char Input() {
  char one_scope;
  std::cin.get(one_scope);
  return one_scope;
}

void CountCorrectScopes(std::vector<int32_t>& stack, int32_t& count_length,
                        int32_t& start_cbs, int32_t& res_length,
                        char one_scope) {
  ++count_length;
  if (one_scope == '(') {
    stack.push_back(count_length);
  } else if (one_scope == ')') {
    if (stack.empty()) {
      start_cbs = count_length;
    } else {
      stack.pop_back();
      if (stack.empty()) {
        res_length = std::max(res_length, count_length - start_cbs);
      } else {
        res_length = std::max(res_length, count_length - stack.back());
      }
    }
  }
}

int main() {
  char one_scope;
  int32_t count_length = 0;
  int32_t res_length = 0;
  int32_t start_cbs = 0;
  std::vector<int32_t> stack;

  while (!std::cin.eof()) {
    one_scope = Input();
    if (one_scope != '(' && one_scope != ')') {
      break;
    }
    CountCorrectScopes(stack, count_length, start_cbs, res_length, one_scope);
  }
  std::cout << res_length;
}
