# Paku Core Tests

This directory contains the test suite for paku-core.

## Test Structure

- `test_payload.cpp` - Unit tests for MQTT payload creation and storage
- `test_intervals.cpp` - Unit tests for interval management logic
- `test_flow.cpp` - Unit tests for flow calculation module
- `test_integration.cpp` - Integration tests for end-to-end data flow
- `native_compat/` - Arduino compatibility layer for native testing (provides String class and other Arduino types for desktop builds)

## Running Tests Locally

### Prerequisites

- CMake 3.14 or later
- C++17 compatible compiler (GCC, Clang, MSVC)
- Git (for fetching Unity test framework)

### Build and Run

```bash
# From the repository root
cd test
mkdir build
cd build
cmake ..
cmake --build .

# Run all tests
ctest --output-on-failure

# Or run individual tests
./test_payload
./test_intervals
./test_flow
./test_integration
```

## Test Coverage

### Unit Tests

1. **Payload Module** (`test_payload.cpp`)
   - JSON payload creation
   - Payload buffer management
   - Buffer overflow protection

2. **Intervals Module** (`test_intervals.cpp`)
   - Heater status-based interval calculation
   - Fast/slow interval transitions
   - Timer reset on heater off

3. **Flow Module** (`test_flow.cpp`)
   - Frequency calculation from pulse count
   - Flow rate calculation
   - Required temperature delta calculation
   - Complete flow data processing

### Integration Tests

1. **Ruuvi Parsing** (`test_integration.cpp`)
   - Valid Ruuvi RAWv2 data parsing
   - Temperature, humidity, pressure extraction
   - Acceleration and battery data
   - Invalid data handling

2. **End-to-End Pipeline**
   - Ruuvi data to MQTT payload conversion
   - Combined sensor data processing
   - Multiple Ruuvi tag simulation

## CI Integration

Tests are automatically run on:
- Push to `main`, `dev`, or `s4/**` branches
- Pull requests to `main` or `dev`

See `.github/workflows/tests.yml` for the CI configuration.

## Adding New Tests

1. Create a new test file (e.g., `test_mymodule.cpp`)
2. Add it to `CMakeLists.txt`:
   ```cmake
   set(UNIT_TEST_SOURCES
       ...
       test_mymodule.cpp
   )
   ```
3. Write tests using Unity framework:
   ```cpp
   #include <unity.h>
   
   void test_my_function(void) {
       TEST_ASSERT_EQUAL(expected, actual);
   }
   
   int main(int argc, char **argv) {
       UNITY_BEGIN();
       RUN_TEST(test_my_function);
       return UNITY_END();
   }
   ```
