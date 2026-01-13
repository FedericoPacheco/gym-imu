# Clean previous coverage data
find .pio/build/host -name "*.gcda" -delete 2>/dev/null

# Run tests
pio test -e host

# Check if tests passed
if [ $? -ne 0 ]; then
  echo "Tests failed, aborting coverage generation."
  exit 1
fi

# Generate coverage report with ignore flags for known issues
lcov --capture --directory .pio/build/host --output-file coverage.info \
  --ignore-errors mismatch,gcov,empty

# Filter out system, dependency, and test code
lcov --remove coverage.info '/usr/*' '*/.pio/libdeps/*' '*/googletest/*' '*/test/*' \
  --output-file coverage_filtered.info \
  --ignore-errors empty,unused

# Check if we have any coverage data
if [ ! -s coverage_filtered.info ]; then
  echo "No coverage data found after filtering."
  exit 1
fi

# Generate HTML report
genhtml coverage_filtered.info --output-directory coverage_report \
  --ignore-errors empty

rm coverage.info coverage_filtered.info

echo "Coverage report generated in coverage_report/index.html"