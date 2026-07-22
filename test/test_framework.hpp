#ifndef TEST_FRAMEWORK_HPP
#define TEST_FRAMEWORK_HPP

#include <cstdio>
#include <cstdint>
#include <cstring>

// Minimal test framework for embedded unit testing
namespace test {

struct TestResult {
    const char* name;
    bool passed;
    const char* file;
    int line;
    const char* message;
};

constexpr uint32_t MAX_TESTS = 128;
extern TestResult results[MAX_TESTS];
extern uint32_t testCount;
extern uint32_t passedCount;
extern uint32_t failedCount;

// Track current test name
extern const char* g_currentTest;

} // namespace test

// Test assertion macros - evaluate cond ONCE to avoid double-evaluation side effects
#define ASSERT_TRUE(cond, msg) \
    do { \
        bool _test_result_ = (cond); \
        test::results[test::testCount].name = test::g_currentTest; \
        test::results[test::testCount].passed = _test_result_; \
        test::results[test::testCount].file = __FILE__; \
        test::results[test::testCount].line = __LINE__; \
        test::results[test::testCount].message = msg; \
        test::testCount++; \
        if (_test_result_) test::passedCount++; else test::failedCount++; \
    } while(0)

#define ASSERT_FALSE(cond, msg) ASSERT_TRUE(!(cond), msg)
#define ASSERT_EQ(a, b, msg) ASSERT_TRUE((a) == (b), msg)
#define ASSERT_NE(a, b, msg) ASSERT_TRUE((a) != (b), msg)
#define ASSERT_NEAR(a, b, eps, msg) ASSERT_TRUE(((a) > (b) - (eps)) && ((a) < (b) + (eps)), msg)
#define ASSERT_GT(a, b, msg) ASSERT_TRUE((a) > (b), msg)
#define ASSERT_LT(a, b, msg) ASSERT_TRUE((a) < (b), msg)
#define ASSERT_GE(a, b, msg) ASSERT_TRUE((a) >= (b), msg)
#define ASSERT_LE(a, b, msg) ASSERT_TRUE((a) <= (b), msg)

// Test definition - creates a function
#define TEST(name) \
    static void test_##name()

// Test runner - call runAllTests() from main
inline int runAllTests() {
    printf("\n========================================\n");
    printf("  STM32H7 AUTONOMY DEMO - UNIT TESTS\n");
    printf("========================================\n\n");
    
    return (test::failedCount == 0) ? 0 : 1;
}

// Print results helper
inline void printResults() {
    printf("\n========================================\n");
    printf("  RESULTS\n");
    printf("========================================\n\n");
    
    for (uint32_t i = 0; i < test::testCount; ++i) {
        const auto& r = test::results[i];
        printf("  [%s] %s\n", r.passed ? "PASS" : "FAIL", r.name);
        if (!r.passed) {
            printf("        %s:%d - %s\n", r.file, r.line, r.message);
        }
    }
    
    printf("\n  Total: %u | Passed: %u | Failed: %u\n\n", 
           test::testCount, test::passedCount, test::failedCount);
    printf("========================================\n");
}

#endif // TEST_FRAMEWORK_HPP
