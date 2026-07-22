#include "test_framework.hpp"

namespace test {

TestResult results[MAX_TESTS];
uint32_t testCount = 0;
uint32_t passedCount = 0;
uint32_t failedCount = 0;
const char* g_currentTest = nullptr;

} // namespace test
