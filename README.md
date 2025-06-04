# Paxcounter M5Stack

This repository contains an Arduino sketch for counting nearby Bluetooth devices. A small unit test is provided for the `truncateString` helper.

## Running the test

1. Install the GoogleTest development package (Ubuntu example):
   ```bash
   sudo apt-get install libgtest-dev googletest
   ```
2. Build the test:
   ```bash
   g++ -std=c++17 truncateString.cpp tests/test_truncateString.cpp \
       -lgtest -lgtest_main -pthread -o tests/test_truncateString
   ```
3. Run it:
   ```bash
   ./tests/test_truncateString
   ```
