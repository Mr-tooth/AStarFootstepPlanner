# Contributing to AStarFootstepPlanner

Thank you for your interest in contributing to AStarFootstepPlanner!

## Development Setup

```bash
# Clone with submodules
git clone --recursive https://github.com/Mr-tooth/AStarFootstepPlanner.git
cd AStarFootstepPlanner

# Build
cmake -B build -DBUILD_TESTING=ON
cmake --build build

# Run tests
ctest --test-dir build
```

## Prerequisites

- CMake 3.22+
- C++11 compliant compiler (GCC 5+, Clang 3.8+, MSVC 2017+)
- [Heuclid v2.0+](https://github.com/Mr-tooth/Heuclid) installed or available via `CMAKE_PREFIX_PATH`
- Eigen 3.3+

## Code Style

- C++11 standard
- Google-style formatting (see `.clang-format`, `IndentWidth: 3`)
- All public APIs should have Doxygen documentation (`/** */`)
- Run `clang-format` before committing

## Pull Request Process

1. Fork and create a feature branch from `main`
2. Ensure all tests pass: `ctest --test-dir build`
3. Add tests for new functionality
4. Update documentation (Doxygen comments, README if needed)
5. Submit PR with a clear description of changes and motivation

## Commit Messages

Follow [Conventional Commits](https://www.conventionalcommits.org/):

| Prefix | Usage |
|--------|-------|
| `feat:` | New feature |
| `fix:` | Bug fix |
| `docs:` | Documentation only |
| `build:` | Build system or dependencies |
| `test:` | Test additions or changes |
| `refactor:` | Code restructuring (no behavior change) |
| `chore:` | Maintenance tasks |

## Reporting Issues

When reporting bugs, please include:

- Compiler and OS version
- CMake version
- Steps to reproduce
- Expected vs. actual behavior

## License

By contributing, you agree that your contributions will be licensed under the [Apache License 2.0](LICENSE).
